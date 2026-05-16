#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import os
import shutil
import signal
import socket
import ssl
import struct
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple
from urllib.parse import urlparse

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:
    serial = None
    list_ports = None

try:
    from pyfiglet import Figlet
    from rich.align import Align
    from rich.console import Group
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    from textual.app import App, ComposeResult
    from textual.binding import Binding
    from textual.containers import Horizontal, Vertical
    from textual.screen import ModalScreen
    from textual.widgets import (
        Button,
        Footer,
        Header,
        Label,
        LoadingIndicator,
        OptionList,
        RichLog,
        Static,
    )
    from textual.widgets.option_list import Option

    UI_IMPORT_ERROR: Optional[Exception] = None
except ImportError as exc:
    UI_IMPORT_ERROR = exc


REPO_ROOT = Path(__file__).resolve().parents[1]
WEBXR_DIR = Path(__file__).resolve().parent
SERVER_DIR = WEBXR_DIR / "server"
CERT_DIR = WEBXR_DIR / "certs"
STATE_PATH = WEBXR_DIR / ".webxr_link_state.json"
RUNTIME_CONFIG_PATH = SERVER_DIR / "runtime-config.json"

DEFAULT_BAUD = 2_000_000
DEFAULT_HTTP_PORT = 8787
DEFAULT_BRIDGE_HOST = "127.0.0.1"
DEFAULT_BRIDGE_PORT = 8791
XR_UART_PACKET_SIZE = 96
XR_PACKET_MAGIC = 0x31525843
XR_PACKET_ABS_POS_OFFSET = 16
XR_PACKET_REL_POS_OFFSET = 28
XR_PACKET_VEC3_X_OFFSET = 0
XR_PACKET_VEC3_Y_OFFSET = 4
XR_BRIDGE_MAGIC = 0x42525843
XR_BRIDGE_VERSION = 1
XR_BRIDGE_HELLO_REQ = 1
XR_BRIDGE_HELLO_RSP = 2
XR_BRIDGE_XR_DATA = 3
XR_BRIDGE_DEVICE_KIND_MASTER = 1
XR_BRIDGE_CAP_PAYLOAD_96 = 0x01
XR_BRIDGE_CAP_DELTA_ARBITER = 0x02
XR_BRIDGE_CAP_CRC16 = 0x04
XR_BRIDGE_HELLO_SIZE = 8
XR_BRIDGE_HEADER_SIZE = 12
XR_BRIDGE_FRAME_OVERHEAD = XR_BRIDGE_HEADER_SIZE + 2
XR_BRIDGE_MAX_PAYLOAD = XR_UART_PACKET_SIZE
MB_OUTPUT_IF_RS232 = 0
MB_OUTPUT_IF_USB = 1
PROCESS_SCAN_PATTERNS = (
    "webxr/server/index.mjs",
    "webxr/webxr_link.py",
    "tools/preview_monitor.py",
    "tools/serial_frame_debug.py",
    "tools/rs232_3d_viewer.py",
)

TUI_SCALE_MIN = 0.65
TUI_SCALE_MAX = 1.35
TUI_DENSITY_VALUES = ("compact", "normal", "comfortable")


@dataclass(frozen=True)
class TuiLayoutConfig:
    scale: float = 1.0
    density: str = "normal"


@dataclass
class XrDeviceStatus:
    detected: bool = False
    bridge_ready: bool = False
    output_if: int = MB_OUTPUT_IF_RS232
    caps: int = 0
    payload_len: int = XR_UART_PACKET_SIZE
    seq: int = 0
    age_ms: int = 0
    raw_info: str = ""


@dataclass
class BridgeSnapshot:
    running: bool = False
    serial_port: str = ""
    serial_baud: int = DEFAULT_BAUD
    serial_connected: bool = False
    client_connected: bool = False
    bridge_host: str = DEFAULT_BRIDGE_HOST
    bridge_port: int = DEFAULT_BRIDGE_PORT
    last_error: str = ""
    xr_status: XrDeviceStatus = field(default_factory=XrDeviceStatus)
    xr_lines: List[str] = field(default_factory=list)


@dataclass
class SerialPortInfo:
    device: str
    description: str
    hwid: str


@dataclass
class AndroidDeviceInfo:
    serial: str
    state: str
    model: str = ""
    detail: str = ""


@dataclass
class CertStatus:
    https_ready: bool
    quest_ca_ready: bool
    server_cert: Path
    server_key: Path
    root_cer: Path
    root_pem: Path
    missing_paths: List[Path] = field(default_factory=list)


@dataclass
class WebServiceStatus:
    running: bool = False
    serve_dist: bool = False
    clients: int = 0
    active_sessions: int = 0
    access_urls: List[str] = field(default_factory=list)
    bridge_connected: bool = False
    bridge_host: str = DEFAULT_BRIDGE_HOST
    bridge_port: int = DEFAULT_BRIDGE_PORT
    latest_frame: Dict[str, object] = field(default_factory=dict)
    last_error: str = ""


@dataclass
class GnirehtetStatus:
    running: bool = False
    device_serial: str = ""
    device_model: str = ""
    last_error: str = ""


@dataclass
class DashboardBundle:
    bridge: BridgeSnapshot
    gnirehtet: GnirehtetStatus
    web: WebServiceStatus
    cert: CertStatus


def normalize_text(value: str) -> str:
    return value.replace("\\", "/").lower()


def relpath(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def print_step(index: int, total: int, title: str) -> None:
    print(f"\n[{index}/{total}] {title}")


def print_note(message: str) -> None:
    print(f"  {message}")


def prompt_yes_no(message: str, default: bool = True) -> bool:
    suffix = " [Y/n] " if default else " [y/N] "
    while True:
        answer = input(message + suffix).strip().lower()
        if not answer:
            return default
        if answer in {"y", "yes"}:
            return True
        if answer in {"n", "no"}:
            return False
        print("请输入 y 或 n。")


def load_json_file(path: Path) -> Dict[str, object]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            return data
    except Exception:
        pass
    return {}


def save_json_file(path: Path, payload: Dict[str, object]) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def load_cli_state() -> Dict[str, object]:
    if not STATE_PATH.exists():
        return {}
    return load_json_file(STATE_PATH)


def save_cli_state(payload: Dict[str, object]) -> None:
    save_json_file(STATE_PATH, payload)


def mark_initial_build_done() -> None:
    payload = load_cli_state()
    payload["initial_build_completed"] = True
    payload["last_build_time"] = int(time.time())
    save_cli_state(payload)


def initial_build_needed() -> bool:
    payload = load_cli_state()
    if not payload.get("initial_build_completed", False):
        return True
    return not (WEBXR_DIR / "dist" / "index.html").exists()


def default_runtime_config() -> Dict[str, object]:
    return {
        "bridgeHost": DEFAULT_BRIDGE_HOST,
        "bridgePort": DEFAULT_BRIDGE_PORT,
        "positionScale": 1.0,
        "rotationScale": 1.0,
        "calibrationModeEnabled": False,
    }


def load_runtime_config() -> Dict[str, object]:
    config = default_runtime_config()
    if RUNTIME_CONFIG_PATH.exists():
        raw = load_json_file(RUNTIME_CONFIG_PATH)
        config.update(raw)
    return config


def save_runtime_config(config: Dict[str, object]) -> None:
    merged = default_runtime_config()
    merged.update(config)
    RUNTIME_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    save_json_file(RUNTIME_CONFIG_PATH, merged)


def clamp_tui_scale(value: object) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return 1.0
    return max(TUI_SCALE_MIN, min(TUI_SCALE_MAX, numeric))


def normalize_tui_density(value: object) -> str:
    density = str(value or "").strip().lower()
    if density in TUI_DENSITY_VALUES:
        return density
    return "normal"


def scaled_cells(base: int, scale: float, minimum: int = 1) -> int:
    return max(minimum, int(round(base * scale)))


def resolve_tui_layout(args: argparse.Namespace) -> TuiLayoutConfig:
    env_scale = os.environ.get("CCWEBXR_TUI_SCALE", "").strip()
    env_density = os.environ.get("CCWEBXR_TUI_DENSITY", "").strip()
    terminal_size = shutil.get_terminal_size(fallback=(160, 48))

    density = normalize_tui_density(getattr(args, "ui_density", "") or env_density)
    scale_source = getattr(args, "ui_scale", None)
    if scale_source is None and env_scale:
        scale_source = env_scale
    scale = clamp_tui_scale(scale_source if scale_source is not None else 1.0)

    if not getattr(args, "ui_density", None) and not env_density:
        if terminal_size.columns < 150 or terminal_size.lines < 42:
            density = "compact"
    if getattr(args, "ui_scale", None) is None and not env_scale:
        if terminal_size.columns < 125 or terminal_size.lines < 34:
            scale = clamp_tui_scale(0.85)

    return TuiLayoutConfig(scale=scale, density=density)


def build_tui_metrics(config: TuiLayoutConfig) -> Dict[str, int]:
    base = {
        "compact": {
            "choice_width": 72,
            "choice_max_height": 24,
            "confirm_width": 68,
            "hero_height": 8,
            "hero_margin_x": 0,
            "main_margin_x": 0,
            "main_margin_bottom": 0,
            "log_pad_left": 0,
            "action_pad_left": 0,
            "action_button_height": 2,
            "busy_height": 1,
        },
        "normal": {
            "choice_width": 88,
            "choice_max_height": 32,
            "confirm_width": 82,
            "hero_height": 12,
            "hero_margin_x": 1,
            "main_margin_x": 1,
            "main_margin_bottom": 1,
            "log_pad_left": 1,
            "action_pad_left": 0,
            "action_button_height": 3,
            "busy_height": 2,
        },
        "comfortable": {
            "choice_width": 96,
            "choice_max_height": 36,
            "confirm_width": 90,
            "hero_height": 14,
            "hero_margin_x": 1,
            "main_margin_x": 1,
            "main_margin_bottom": 1,
            "log_pad_left": 1,
            "action_pad_left": 1,
            "action_button_height": 4,
            "busy_height": 2,
        },
    }[config.density]
    return {
        key: scaled_cells(value, config.scale, 0 if key.endswith(("_x", "_bottom", "_left")) else 1)
        for key, value in base.items()
    }


def build_choice_screen_css(config: TuiLayoutConfig) -> str:
    metrics = build_tui_metrics(config)
    return f"""
        ChoiceScreen {{
            align: center middle;
            background: rgba(2, 6, 23, 0.82);
        }}
        #choice_dialog {{
            width: {metrics["choice_width"]};
            height: auto;
            max-height: {metrics["choice_max_height"]};
            background: #0f172a;
            border: thick #38bdf8;
            padding: 1 2;
        }}
        #choice_title {{
            color: #f8fafc;
            text-style: bold;
            margin-bottom: 1;
        }}
        #choice_desc {{
            color: #94a3b8;
            margin-bottom: 1;
        }}
        #choice_buttons {{
            margin-top: 1;
            height: auto;
        }}
        #choice_buttons Button {{
            width: 1fr;
            margin-right: 1;
        }}
        """


def build_confirm_screen_css(config: TuiLayoutConfig) -> str:
    metrics = build_tui_metrics(config)
    return f"""
        ConfirmScreen {{
            align: center middle;
            background: rgba(2, 6, 23, 0.78);
        }}
        #confirm_dialog {{
            width: {metrics["confirm_width"]};
            height: auto;
            background: #0f172a;
            border: thick #22c55e;
            padding: 1 2;
        }}
        #confirm_title {{
            color: #f8fafc;
            text-style: bold;
            margin-bottom: 1;
        }}
        #confirm_message {{
            color: #cbd5e1;
            margin-bottom: 1;
        }}
        #confirm_buttons {{
            height: auto;
        }}
        #confirm_buttons Button {{
            width: 1fr;
            margin-right: 1;
        }}
        """


def build_app_css(config: TuiLayoutConfig) -> str:
    metrics = build_tui_metrics(config)
    return f"""
        Screen {{
            background: #060816;
            color: #e2e8f0;
        }}
        #root {{
            layout: vertical;
            height: 1fr;
        }}
        #hero {{
            height: {metrics["hero_height"]};
            margin: 0 {metrics["hero_margin_x"]} 0 {metrics["hero_margin_x"]};
        }}
        #main {{
            layout: horizontal;
            height: 1fr;
            margin: 0 {metrics["main_margin_x"]} {metrics["main_margin_bottom"]} {metrics["main_margin_x"]};
        }}
        #status_title, #log_title, #action_title {{
            color: #f8fafc;
            text-style: bold;
            margin-bottom: 0;
        }}
        #status_column {{
            width: 5fr;
            padding-right: 0;
        }}
        #log_column {{
            width: 4fr;
            padding: 0 0 0 {metrics["log_pad_left"]};
        }}
        #action_column {{
            width: 3fr;
            padding-left: {metrics["action_pad_left"]};
        }}
        .status_card {{
            height: auto;
            margin-bottom: 0;
        }}
        #event_log {{
            border: round #334155;
            background: #0b1120;
            color: #dbeafe;
            height: 1fr;
        }}
        #busy_indicator {{
            height: {metrics["busy_height"]};
            margin-bottom: 0;
        }}
        #step_status {{
            margin-bottom: 0;
        }}
        #action_column Button {{
            width: 100%;
            height: {metrics["action_button_height"]};
            min-height: {metrics["action_button_height"]};
            margin-bottom: 0;
        }}
        Footer {{
            background: #0f172a;
        }}
        """


def list_ipv4_hosts() -> List[str]:
    results: List[str] = ["127.0.0.1", "localhost"]
    seen = set(results)

    try:
        hostname = socket.gethostname()
        for _family, _type, _proto, _canon, sockaddr in socket.getaddrinfo(
            hostname,
            None,
            socket.AF_INET,
        ):
            host = sockaddr[0]
            if host not in seen:
                seen.add(host)
                results.append(host)
    except socket.gaierror:
        pass

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("192.0.2.1", 1))
            host = sock.getsockname()[0]
            if host and host not in seen:
                seen.add(host)
                results.append(host)
    except OSError:
        pass

    return results


def build_access_urls(port: int, scheme: str) -> List[str]:
    return [f"{scheme}://{host}:{port}" for host in list_ipv4_hosts()]


def pick_remote_access_url(urls: Sequence[str]) -> Optional[str]:
    for url in urls:
        try:
            host = (urlparse(url).hostname or "").strip().lower()
        except Exception:
            continue
        if host and host not in {"127.0.0.1", "localhost", "::1", "[::1]", "0.0.0.0"}:
            return url
    return None


def resolve_executable(local_candidates: Sequence[Path], names: Sequence[str]) -> Optional[str]:
    for candidate in local_candidates:
        if candidate.exists():
            return str(candidate)
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    return None


def resolve_node_executable() -> Optional[str]:
    local: List[Path] = []
    if os.name == "nt":
        local.append(WEBXR_DIR / "runtime" / "node" / "node.exe")
    else:
        local.append(WEBXR_DIR / "runtime" / "node" / "bin" / "node")
        local.append(WEBXR_DIR / "runtime" / "node" / "node")
    return resolve_executable(
        local,
        ("node.exe", "node") if os.name == "nt" else ("node",),
    )


def resolve_npm_executable() -> Optional[str]:
    local: List[Path] = []
    if os.name == "nt":
        local.append(WEBXR_DIR / "runtime" / "node" / "npm.cmd")
    else:
        local.append(WEBXR_DIR / "runtime" / "node" / "bin" / "npm")
    return resolve_executable(
        local,
        ("npm.cmd", "npm") if os.name == "nt" else ("npm",),
    )


def resolve_adb_executable() -> Optional[str]:
    local = []
    if os.name == "nt":
        local.append(WEBXR_DIR / "platform-tools" / "adb.exe")
    else:
        local.append(WEBXR_DIR / "platform-tools" / "adb")
    names = ("adb.exe", "adb") if os.name == "nt" else ("adb",)
    return resolve_executable(local, names)


def resolve_gnirehtet_executable() -> Optional[str]:
    local = []
    if os.name == "nt":
        local.append(WEBXR_DIR / "platform-tools" / "gnirehtet.exe")
    else:
        local.append(WEBXR_DIR / "platform-tools" / "gnirehtet")
    names = ("gnirehtet.exe", "gnirehtet") if os.name == "nt" else ("gnirehtet",)
    return resolve_executable(local, names)


def run_command(
    args: Sequence[str],
    *,
    cwd: Path = WEBXR_DIR,
    timeout: Optional[float] = None,
) -> Tuple[bool, str]:
    try:
        completed = subprocess.run(
            list(args),
            cwd=str(cwd),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            shell=False,
            timeout=timeout,
            check=False,
        )
    except Exception as exc:
        return False, str(exc)

    output = ((completed.stdout or "") + (completed.stderr or "")).strip()
    if completed.returncode != 0:
        return False, output or f"命令失败: {' '.join(args)}"
    return True, output or "OK"


def fetch_json(url: str) -> Dict[str, object]:
    request = urllib.request.Request(url, headers={"User-Agent": "WebXRLink/1.0"})
    kwargs: Dict[str, object] = {}
    if url.startswith("https://"):
        kwargs["context"] = ssl._create_unverified_context()
    with urllib.request.urlopen(request, timeout=2.0, **kwargs) as response:
        payload = json.loads(response.read().decode("utf-8"))
    return payload if isinstance(payload, dict) else {}


def probe_local_status(port: int) -> Tuple[str, Dict[str, object]]:
    candidates = (
        f"https://127.0.0.1:{port}/status",
        f"http://127.0.0.1:{port}/status",
    )
    last_error = "unknown"
    for url in candidates:
        try:
            return url, fetch_json(url)
        except Exception as exc:
            last_error = str(exc)
    raise RuntimeError(last_error)


def rm_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
        crc &= 0xFFFF
    return crc


def build_bridge_frame(frame_type: int, seq: int, payload: bytes = b"") -> bytes:
    payload = payload or b""
    header = struct.pack(
        "<IBBHI",
        XR_BRIDGE_MAGIC,
        XR_BRIDGE_VERSION,
        int(frame_type) & 0xFF,
        len(payload),
        int(seq) & 0xFFFFFFFF,
    )
    frame_wo_crc = header + payload
    crc = rm_crc16(frame_wo_crc)
    return frame_wo_crc + struct.pack("<H", crc)


def remap_packet_world_position_for_esp32(packet: bytes) -> bytes:
    if len(packet) != XR_UART_PACKET_SIZE:
        return packet
    try:
        magic, version, _flags, _seq = struct.unpack_from("<IHHI", packet, 0)
    except struct.error:
        return packet
    if magic != XR_PACKET_MAGIC or version != 1:
        return packet

    mutable = bytearray(packet)
    for base_offset in (XR_PACKET_ABS_POS_OFFSET, XR_PACKET_REL_POS_OFFSET):
        x_offset = base_offset + XR_PACKET_VEC3_X_OFFSET
        x_value = struct.unpack_from("<f", mutable, x_offset)[0]
        # Keep one host-side hook for ESP32 world-frame tweaks. The current
        # contract needs the X axis flipped back to the raw WebXR packet sign.
        struct.pack_into("<f", mutable, x_offset, x_value)
    return bytes(mutable)


def is_transient_serial_write_error(exc: Exception) -> bool:
    timeout_type = getattr(serial, "SerialTimeoutException", None) if serial is not None else None
    if timeout_type is not None and isinstance(exc, timeout_type):
        return True
    text = str(exc).strip().lower()
    return "write timeout" in text or ("timed out" in text and "write" in text)


def parse_bridge_hello_payload(payload: bytes, seq: int) -> Optional[XrDeviceStatus]:
    if len(payload) != XR_BRIDGE_HELLO_SIZE:
        return None
    device_kind, output_if, bridge_ready, caps, payload_len, _reserved = struct.unpack(
        "<BBBBHH", payload
    )
    if device_kind != XR_BRIDGE_DEVICE_KIND_MASTER:
        return None
    return XrDeviceStatus(
        detected=True,
        bridge_ready=bool(bridge_ready),
        output_if=int(output_if),
        caps=int(caps),
        payload_len=int(payload_len),
        seq=int(seq),
        age_ms=0,
        raw_info=(
            f"out_if={int(output_if)} ready={int(bool(bridge_ready))} "
            f"caps=0x{int(caps):02X} payload={int(payload_len)}"
        ),
    )


def list_serial_ports() -> List[SerialPortInfo]:
    if list_ports is None:
        return []
    ports: List[SerialPortInfo] = []
    for info in list_ports.comports():
        ports.append(
            SerialPortInfo(
                device=str(info.device),
                description=str(info.description or "未知设备"),
                hwid=str(info.hwid or ""),
            )
        )
    return ports


def describe_serial_port(port: SerialPortInfo) -> str:
    tail = f" | {port.hwid}" if port.hwid else ""
    return f"{port.device} | {port.description}{tail}"


def choose_serial_port(preferred: str = "") -> Optional[SerialPortInfo]:
    ports = list_serial_ports()
    if not ports:
        print_note("未扫描到可用串口。")
        return None

    if preferred:
        for port in ports:
            if port.device == preferred:
                print_note(f"使用命令行指定串口: {describe_serial_port(port)}")
                return port
        print_note(f"命令行指定串口 {preferred} 不在当前扫描结果中，将转为手动选择。")

    print_note("可用串口:")
    for index, port in enumerate(ports, start=1):
        print(f"  {index}. {describe_serial_port(port)}")
    print("  0. 跳过")

    while True:
        answer = input("请选择 CCtrl USB 串口编号: ").strip()
        if answer == "0":
            return None
        if answer.isdigit():
            index = int(answer)
            if 1 <= index <= len(ports):
                return ports[index - 1]
        print("请输入有效编号。")


def describe_android_device(device: AndroidDeviceInfo) -> str:
    pieces = [device.serial, device.state]
    if device.model:
        pieces.append(device.model)
    if device.detail:
        pieces.append(device.detail)
    return " | ".join(pieces)


def list_process_entries() -> List[Tuple[int, str]]:
    if os.name == "nt":
        return list_process_entries_windows()
    return list_process_entries_posix()


def list_process_entries_windows() -> List[Tuple[int, str]]:
    script = (
        "Get-CimInstance Win32_Process | "
        "Select-Object ProcessId,Name,CommandLine | ConvertTo-Json -Compress"
    )
    ok, output = run_command(
        ("powershell", "-NoProfile", "-Command", script),
        cwd=REPO_ROOT,
        timeout=15,
    )
    if not ok or not output:
        return []
    try:
        raw = json.loads(output)
    except json.JSONDecodeError:
        return []

    entries = raw if isinstance(raw, list) else [raw]
    result: List[Tuple[int, str]] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        pid = int(entry.get("ProcessId", 0) or 0)
        command_line = str(entry.get("CommandLine", "") or "")
        name = str(entry.get("Name", "") or "")
        if pid > 0:
            result.append((pid, command_line or name))
    return result


def list_process_entries_posix() -> List[Tuple[int, str]]:
    ok, output = run_command(("ps", "-eo", "pid=,command="), cwd=REPO_ROOT, timeout=15)
    if not ok or not output:
        return []
    result: List[Tuple[int, str]] = []
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        result.append((pid, parts[1]))
    return result


def terminate_pid(pid: int) -> bool:
    if pid <= 0 or pid == os.getpid():
        return False
    try:
        if os.name == "nt":
            subprocess.run(
                ("taskkill", "/PID", str(pid), "/T", "/F"),
                capture_output=True,
                text=True,
                check=False,
            )
        else:
            os.kill(pid, signal.SIGTERM)
    except Exception:
        return False
    return True


def cleanup_residual_processes() -> List[str]:
    killed: List[str] = []
    for pid, command in list_process_entries():
        if pid == os.getpid():
            continue
        normalized = normalize_text(command)
        is_match = any(pattern in normalized for pattern in PROCESS_SCAN_PATTERNS)
        is_gnirehtet = "gnirehtet" in normalized and (
            " run" in normalized
            or " start" in normalized
            or " autorun" in normalized
            or normalized.endswith("/gnirehtet")
            or normalized.endswith("\\gnirehtet.exe")
        )
        if not is_match and not is_gnirehtet:
            continue
        if terminate_pid(pid):
            killed.append(f"{pid}: {command}")
    return killed


def stop_process_tree(process: subprocess.Popen[str], timeout: float = 5.0) -> None:
    if process.poll() is not None:
        return

    try:
        if os.name == "nt":
            process.terminate()
        else:
            os.killpg(process.pid, signal.SIGTERM)
    except Exception:
        try:
            process.terminate()
        except Exception:
            pass

    try:
        process.wait(timeout=timeout)
        return
    except Exception:
        pass

    try:
        if os.name == "nt":
            subprocess.run(
                ("taskkill", "/PID", str(process.pid), "/T", "/F"),
                capture_output=True,
                text=True,
                check=False,
            )
        else:
            os.killpg(process.pid, signal.SIGKILL)
    except Exception:
        try:
            process.kill()
        except Exception:
            pass


class XrUartBridgeManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._thread_rx: Optional[threading.Thread] = None
        self._thread_tx: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._serial = None
        self._serial_port = ""
        self._serial_baud = DEFAULT_BAUD
        self._bridge_host = DEFAULT_BRIDGE_HOST
        self._bridge_port = DEFAULT_BRIDGE_PORT
        self._running = False
        self._serial_connected = False
        self._client_connected = False
        self._last_error = ""
        self._xr_status = XrDeviceStatus()
        self._xr_lines: Deque[str] = deque(maxlen=64)
        self._status_events: Deque[Tuple[int, XrDeviceStatus]] = deque(maxlen=32)
        self._status_event_id = 0
        self._packet_buffer = bytearray()
        self._device_buffer = bytearray()
        self._packet_queue: Deque[bytes] = deque(maxlen=8)
        self._pending_packet = b""
        self._next_tx_seq = 1
        self._need_handshake = True
        self._last_handshake_sent_at = 0.0
        self._last_status_received_at = 0.0
        self._serial_write_failures = 0

    @staticmethod
    def _quiesce_serial_control_lines(ser: object) -> None:
        for attr_name in ("dsrdtr", "rtscts", "dtr", "rts"):
            try:
                setattr(ser, attr_name, False)
            except Exception:
                pass
        for method_name in ("setDTR", "setRTS"):
            try:
                getattr(ser, method_name)(False)
            except Exception:
                pass

    def _open_serial_port(self, port: str, baud: int):
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.01
        ser.write_timeout = 0.5
        self._quiesce_serial_control_lines(ser)
        ser.open()
        self._quiesce_serial_control_lines(ser)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        try:
            ser.reset_output_buffer()
        except Exception:
            pass
        return ser

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def start(self, serial_port: str, serial_baud: int, bridge_host: str, bridge_port: int) -> Tuple[bool, str]:
        serial_port = serial_port.strip()
        if not serial_port:
            return False, "请先选择 CCtrl USB 串口。"
        if serial is None:
            return False, "缺少 pyserial，请先安装 pyserial。"

        self.stop()
        self._stop_event = threading.Event()

        with self._lock:
            self._serial_port = serial_port
            self._serial_baud = serial_baud
            self._bridge_host = bridge_host
            self._bridge_port = bridge_port
            self._running = True
            self._serial_connected = False
            self._client_connected = False
            self._last_error = ""
            self._packet_buffer.clear()
            self._device_buffer.clear()
            self._packet_queue.clear()
            self._pending_packet = b""
            self._xr_status = XrDeviceStatus()
            self._status_events.clear()
            self._status_event_id = 0
            self._next_tx_seq = 1
            self._need_handshake = True
            self._last_handshake_sent_at = 0.0
            self._last_status_received_at = 0.0
            self._serial_write_failures = 0

        self._thread_rx = threading.Thread(target=self._run_loop, daemon=True)
        self._thread_tx = threading.Thread(target=self._tx_loop, daemon=True)
        self._thread_rx.start()
        self._thread_tx.start()
        return True, f"XR 串口桥已启动: {serial_port}"

    def stop(self) -> None:
        self._stop_event.set()
        thread_rx = self._thread_rx
        thread_tx = self._thread_tx
        if thread_rx is not None:
            thread_rx.join(timeout=1.5)
        if thread_tx is not None:
            thread_tx.join(timeout=1.5)
        with self._lock:
            self._thread_rx = None
            self._thread_tx = None
            self._running = False

    def wait_for_serial_ready(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            if snapshot.serial_connected:
                return True
            time.sleep(0.05)
        return self.snapshot().serial_connected

    def status_marker(self) -> int:
        with self._lock:
            return self._status_event_id

    def wait_for_status(
        self,
        predicate: Callable[[XrDeviceStatus], bool],
        timeout: float,
        *,
        after_id: Optional[int] = None,
    ) -> Optional[XrDeviceStatus]:
        deadline = time.monotonic() + timeout
        with self._condition:
            start_id = self._status_event_id if after_id is None else after_id
            while True:
                for event_id, status in self._status_events:
                    if event_id > start_id and predicate(status):
                        return XrDeviceStatus(**vars(status))
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)

    def request_status(self, timeout: float = 1.5) -> Optional[XrDeviceStatus]:
        marker = self.status_marker()
        if not self._send_hello_request():
            return None
        return self.wait_for_status(
            lambda _item: True,
            timeout,
            after_id=marker,
        )

    def enter_xr_mode(self, timeout: float = 2.0) -> Tuple[bool, str]:
        status = self.request_status(timeout=timeout)
        if status is None:
            return False, self.snapshot().last_error or "等待桥接握手超时。"
        if status.bridge_ready:
            return True, "下位机已识别，XR 桥接可用。"
        output_name = "USB" if status.output_if == MB_OUTPUT_IF_USB else "RS232"
        return False, f"已识别设备，但当前 LinkOut={output_name}，请切换到 RS232。"

    def exit_xr_mode(self, timeout: float = 3.0) -> Tuple[bool, str]:
        _ = timeout
        return True, "当前桥接架构无需退出 XR 模式。"

    def snapshot(self) -> BridgeSnapshot:
        with self._lock:
            status = XrDeviceStatus(**vars(self._xr_status))
            if status.detected and self._last_status_received_at > 0:
                status.age_ms = int(max(0.0, (time.monotonic() - self._last_status_received_at) * 1000.0))
            return BridgeSnapshot(
                running=self._running,
                serial_port=self._serial_port,
                serial_baud=self._serial_baud,
                serial_connected=self._serial_connected,
                client_connected=self._client_connected,
                bridge_host=self._bridge_host,
                bridge_port=self._bridge_port,
                last_error=self._last_error,
                xr_status=status,
                xr_lines=list(self._xr_lines),
            )

    def _record_note(self, line: str) -> None:
        text = line.strip()
        if not text:
            return
        with self._condition:
            self._xr_lines.append(text)
            self._condition.notify_all()

    def _record_status(self, status: XrDeviceStatus) -> None:
        with self._condition:
            status.age_ms = 0
            self._xr_status = status
            self._status_event_id += 1
            self._status_events.append((self._status_event_id, XrDeviceStatus(**vars(status))))
            self._last_status_received_at = time.monotonic()
            self._need_handshake = not status.bridge_ready
            self._xr_lines.append(
                f"HELLO seq={status.seq} ready={int(status.bridge_ready)} "
                f"out_if={'USB' if status.output_if == MB_OUTPUT_IF_USB else 'RS232'} "
                f"payload={status.payload_len}"
            )
            self._condition.notify_all()

    def _next_seq(self) -> int:
        with self._lock:
            seq = self._next_tx_seq
            self._next_tx_seq = (self._next_tx_seq + 1) & 0xFFFFFFFF
            if self._next_tx_seq == 0:
                self._next_tx_seq = 1
            return seq

    def _send_hello_request(self) -> bool:
        with self._lock:
            ser = self._serial
        if ser is None:
            return False
        payload = build_bridge_frame(XR_BRIDGE_HELLO_REQ, self._next_seq())
        with self._lock:
            self._last_handshake_sent_at = time.monotonic()
        try:
            ser.write(payload)
            with self._lock:
                self._serial_write_failures = 0
            return True
        except Exception as exc:
            self._handle_serial_write_exception(exc)
            return False

    def _ensure_serial(self) -> None:
        if serial is None:
            with self._lock:
                self._last_error = "缺少 pyserial"
            time.sleep(0.2)
            return

        with self._lock:
            if self._serial is not None:
                return
            port = self._serial_port
            baud = self._serial_baud

        try:
            ser = self._open_serial_port(port, baud)
        except Exception as exc:
            with self._lock:
                self._serial_connected = False
                self._last_error = str(exc)
            time.sleep(0.2)
            return

        with self._lock:
            self._serial = ser
            self._serial_connected = True
            self._last_error = ""
            self._need_handshake = True
            self._device_buffer.clear()
            self._packet_buffer.clear()
            self._serial_write_failures = 0

    def _close_serial(self) -> None:
        with self._lock:
            ser = self._serial
            self._serial = None
            self._serial_connected = False
            self._need_handshake = True
            self._xr_status = XrDeviceStatus()
            self._last_status_received_at = 0.0
            self._serial_write_failures = 0
        if ser is not None:
            try:
                self._quiesce_serial_control_lines(ser)
            except Exception:
                pass
            try:
                ser.close()
            except Exception:
                pass

    def _handle_serial_write_exception(self, exc: Exception, packet: bytes = b"") -> bool:
        should_close = True
        failures = 0
        transient = is_transient_serial_write_error(exc)
        with self._lock:
            self._last_error = str(exc)
            if packet:
                self._pending_packet = packet
            if transient:
                self._serial_write_failures += 1
                failures = self._serial_write_failures
                should_close = failures >= 6
            else:
                self._serial_write_failures = 0
        if transient and not should_close:
            self._record_note(f"USB write timeout x{failures}, keep bridge open and retry")
            return False
        self._close_serial()
        return True

    def _forward_packet(self, packet: bytes) -> None:
        with self._lock:
            ser = self._serial
            status = XrDeviceStatus(**vars(self._xr_status))
        if ser is None or not status.bridge_ready:
            with self._lock:
                self._pending_packet = packet
            return

        magic, version, _flags, inner_seq = struct.unpack_from("<IHHI", packet, 0)
        if magic != XR_PACKET_MAGIC or version != 1:
            self._record_note("XR 二进制包格式错误")
            return

        esp32_packet = remap_packet_world_position_for_esp32(packet)
        payload = build_bridge_frame(XR_BRIDGE_XR_DATA, int(inner_seq), esp32_packet)
        try:
            ser.write(payload)
            with self._lock:
                self._pending_packet = b""
                self._serial_write_failures = 0
        except Exception as exc:
            self._handle_serial_write_exception(exc, packet)

    def _flush_pending_packet(self) -> None:
        with self._lock:
            packet = self._pending_packet
            ready = self._serial is not None and self._xr_status.bridge_ready
        if ready and packet:
            self._forward_packet(packet)

    def _consume_bridge_bytes(self, chunk: bytes) -> None:
        if not chunk:
            return
        self._packet_buffer.extend(chunk)
        while len(self._packet_buffer) >= XR_UART_PACKET_SIZE:
            packet = bytes(self._packet_buffer[:XR_UART_PACKET_SIZE])
            del self._packet_buffer[:XR_UART_PACKET_SIZE]
            magic, version, _flags, seq = struct.unpack_from("<IHHI", packet, 0)
            if magic != XR_PACKET_MAGIC or version != 1:
                with self._lock:
                    self._last_error = "XR 二进制包格式错误"
                continue
            with self._lock:
                self._packet_queue.append(packet)

    def _consume_serial_bridge_frames(self, chunk: bytes) -> None:
        if not chunk:
            return
        magic = struct.pack("<I", XR_BRIDGE_MAGIC)
        self._device_buffer.extend(chunk)
        while True:
            index = self._device_buffer.find(magic)
            if index < 0:
                if len(self._device_buffer) > 3:
                    del self._device_buffer[:-3]
                return
            if index > 0:
                del self._device_buffer[:index]
            if len(self._device_buffer) < XR_BRIDGE_HEADER_SIZE:
                return

            frame_magic, version, frame_type, payload_len, seq = struct.unpack_from(
                "<IBBHI", self._device_buffer, 0
            )
            if frame_magic != XR_BRIDGE_MAGIC or version != XR_BRIDGE_VERSION or payload_len > XR_BRIDGE_MAX_PAYLOAD:
                del self._device_buffer[0]
                continue

            frame_len = XR_BRIDGE_HEADER_SIZE + int(payload_len) + 2
            if len(self._device_buffer) < frame_len:
                return

            frame = bytes(self._device_buffer[:frame_len])
            recv_crc = struct.unpack_from("<H", frame, frame_len - 2)[0]
            calc_crc = rm_crc16(frame[:-2])
            if recv_crc != calc_crc:
                del self._device_buffer[0]
                continue

            payload = frame[XR_BRIDGE_HEADER_SIZE:-2]
            del self._device_buffer[:frame_len]
            if frame_type == XR_BRIDGE_HELLO_RSP:
                status = parse_bridge_hello_payload(payload, int(seq))
                if status is not None:
                    self._record_status(status)

    def _poll_handshake(self) -> None:
        with self._lock:
            serial_ready = self._serial is not None
            status = XrDeviceStatus(**vars(self._xr_status))
            last_sent_at = self._last_handshake_sent_at
            last_recv_at = self._last_status_received_at
            need = self._need_handshake or not status.detected or not status.bridge_ready
        if not serial_ready:
            return
        now = time.monotonic()
        if not need and (now - last_recv_at) < 2.0:
            return
        if (now - last_sent_at) < 0.5:
            return
        self._send_hello_request()

    def _tx_loop(self) -> None:
        target_interval = 1.0 / 60.0
        while not self._stop_event.is_set():
            start_time = time.perf_counter()
            packet: Optional[bytes] = None
            with self._lock:
                if self._packet_queue:
                    packet = self._packet_queue[-1]
                    self._packet_queue.clear()
            if packet is not None:
                self._forward_packet(packet)
            elapsed = time.perf_counter() - start_time
            remaining = target_interval - elapsed
            if remaining > 0:
                end_time = time.perf_counter() + remaining
                while time.perf_counter() < end_time:
                    time.sleep(0)

    def _run_loop(self) -> None:
        server_socket = None
        client_socket = None
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            with self._lock:
                bind_host = self._bridge_host
                bind_port = self._bridge_port
            server_socket.bind((bind_host, bind_port))
            server_socket.listen(1)
            server_socket.settimeout(0.05)

            while not self._stop_event.is_set():
                self._ensure_serial()
                self._poll_handshake()
                self._flush_pending_packet()

                if client_socket is None:
                    try:
                        candidate, _addr = server_socket.accept()
                        candidate.settimeout(0.02)
                        candidate.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        client_socket = candidate
                        with self._lock:
                            self._client_connected = True
                            self._last_error = ""
                    except socket.timeout:
                        pass
                    except Exception as exc:
                        with self._lock:
                            self._last_error = str(exc)
                        time.sleep(0.05)
                else:
                    try:
                        chunk = client_socket.recv(4096)
                        if chunk:
                            self._consume_bridge_bytes(chunk)
                        else:
                            client_socket.close()
                            client_socket = None
                            with self._lock:
                                self._client_connected = False
                    except socket.timeout:
                        pass
                    except Exception as exc:
                        with self._lock:
                            self._last_error = str(exc)
                            self._client_connected = False
                        try:
                            client_socket.close()
                        except Exception:
                            pass
                        client_socket = None

                with self._lock:
                    ser = self._serial
                if ser is not None:
                    try:
                        chunk = ser.read(ser.in_waiting or 1)
                        if chunk:
                            self._consume_serial_bridge_frames(chunk)
                    except Exception as exc:
                        with self._lock:
                            self._last_error = str(exc)
                        self._close_serial()
        finally:
            if client_socket is not None:
                try:
                    client_socket.close()
                except Exception:
                    pass
            if server_socket is not None:
                try:
                    server_socket.close()
                except Exception:
                    pass
            self._close_serial()
            with self._lock:
                self._client_connected = False
                self._running = False


class WebServiceManager:
    def __init__(self, http_port: int) -> None:
        self._http_port = http_port
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen[str]] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._last_error = ""
        self._node_path = resolve_node_executable()
        self._npm_path = resolve_npm_executable()

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def is_running(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def _ensure_node(self) -> Tuple[bool, str]:
        if self._node_path:
            return True, self._node_path
        return False, "未找到 node，请先安装 Node.js 并确保 node 在 PATH 中。"

    def _ensure_npm(self) -> Tuple[bool, str]:
        if self._npm_path:
            return True, self._npm_path
        return False, "未找到 npm，请先安装 Node.js 并确保 npm 在 PATH 中。"

    def build_frontend(self) -> Tuple[bool, str]:
        if not (WEBXR_DIR / "node_modules").exists():
            return False, "未找到 webxr/node_modules，请先执行 npm install。"
        ok, npm_or_message = self._ensure_npm()
        if not ok:
            return False, npm_or_message
        return run_command((npm_or_message, "run", "build"), cwd=WEBXR_DIR, timeout=300)

    def generate_certs(self) -> Tuple[bool, str]:
        ok, node_or_message = self._ensure_node()
        if not ok:
            return False, node_or_message
        return run_command(
            (node_or_message, str(WEBXR_DIR / "scripts" / "generate-dev-certs.mjs")),
            cwd=WEBXR_DIR,
            timeout=180,
        )

    def start_service(self, build_first: bool = False) -> Tuple[bool, str]:
        if self.is_running():
            return True, "WebXR 服务已在运行。"
        if build_first:
            ok, message = self.build_frontend()
            if not ok:
                self._set_error(message)
                return False, message

        ok, node_or_message = self._ensure_node()
        if not ok:
            self._set_error(node_or_message)
            return False, node_or_message

        creation_flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0) if os.name == "nt" else 0
        try:
            process = subprocess.Popen(
                (node_or_message, str(WEBXR_DIR / "server" / "index.mjs"), "--serve-dist"),
                cwd=str(WEBXR_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                shell=False,
                start_new_session=(os.name != "nt"),
                creationflags=creation_flags,
            )
        except Exception as exc:
            self._set_error(str(exc))
            return False, str(exc)

        with self._lock:
            self._process = process
            self._last_error = ""

        self._reader_thread = threading.Thread(target=self._read_stdout, daemon=True)
        self._reader_thread.start()

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if not self.is_running():
                return False, self.last_error() or "WebXR 服务异常退出。"
            try:
                _url, _payload = probe_local_status(self._http_port)
                return True, "WebXR 服务启动成功。"
            except Exception:
                time.sleep(0.15)
        return False, self.last_error() or "等待 WebXR 服务启动超时。"

    def stop_service(self) -> None:
        with self._lock:
            process = self._process
            self._process = None
        if process is not None:
            stop_process_tree(process)

    def _read_stdout(self) -> None:
        with self._lock:
            process = self._process
        if process is None or process.stdout is None:
            return
        try:
            for _line in process.stdout:
                pass
        finally:
            code = process.poll()
            if code not in (None, 0):
                self._set_error(f"WebXR 服务退出，返回码 {code}")

    def fetch_status(self) -> WebServiceStatus:
        config = load_runtime_config()
        status = WebServiceStatus(
            running=self.is_running(),
            bridge_host=str(config.get("bridgeHost", DEFAULT_BRIDGE_HOST) or DEFAULT_BRIDGE_HOST),
            bridge_port=int(config.get("bridgePort", DEFAULT_BRIDGE_PORT) or DEFAULT_BRIDGE_PORT),
            access_urls=build_access_urls(self._http_port, "https"),
            last_error=self.last_error(),
        )
        if not status.running:
            cert_status = collect_cert_status()
            scheme = "https" if cert_status.https_ready else "http"
            status.access_urls = build_access_urls(self._http_port, scheme)
            return status

        try:
            _url, payload = probe_local_status(self._http_port)
            status.serve_dist = bool(payload.get("serveDist", False))
            status.clients = int(payload.get("clients", 0) or 0)
            status.active_sessions = int(payload.get("activeSessions", 0) or 0)

            relay = payload.get("relay", {})
            if isinstance(relay, dict):
                status.bridge_connected = bool(relay.get("bridgeConnected", False))
                status.bridge_host = str(relay.get("bridgeHost", status.bridge_host) or status.bridge_host)
                status.bridge_port = int(relay.get("bridgePort", status.bridge_port) or status.bridge_port)
                latest = relay.get("latestFrame", {})
                status.latest_frame = latest if isinstance(latest, dict) else {}
                status.last_error = str(relay.get("lastBridgeError", "") or status.last_error)

            urls = payload.get("accessUrls", [])
            if isinstance(urls, list) and urls:
                status.access_urls = [str(item) for item in urls if item]
            else:
                cert_status = collect_cert_status()
                scheme = "https" if cert_status.https_ready else "http"
                status.access_urls = build_access_urls(self._http_port, scheme)
        except Exception as exc:
            status.last_error = str(exc)
            cert_status = collect_cert_status()
            scheme = "https" if cert_status.https_ready else "http"
            status.access_urls = build_access_urls(self._http_port, scheme)
        return status


class GnirehtetManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen[str]] = None
        self._thread: Optional[threading.Thread] = None
        self._last_error = ""
        self._device: Optional[AndroidDeviceInfo] = None
        self._stop_requested = False
        self._adb_path = resolve_adb_executable()
        self._gnirehtet_path = resolve_gnirehtet_executable()

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def _ensure_adb(self) -> Tuple[bool, str]:
        if self._adb_path:
            return True, self._adb_path
        return False, "未找到 adb。Windows 下请检查 webxr/platform-tools，Arch Linux 下请安装 adb 并加入 PATH。"

    def _ensure_gnirehtet(self) -> Tuple[bool, str]:
        if self._gnirehtet_path:
            return True, self._gnirehtet_path
        return False, "未找到 gnirehtet。Windows 下请检查 webxr/platform-tools，Arch Linux 下请安装 gnirehtet 并加入 PATH。"

    def _run_adb_for_device(self, adb_args: Sequence[str], *, timeout: float = 20.0) -> Tuple[bool, str]:
        ok, adb_or_message = self._ensure_adb()
        if not ok:
            return False, adb_or_message
        with self._lock:
            device = self._device
        if device is None:
            return False, "当前没有可用的 Quest/Android 设备。"
        return run_command((adb_or_message, "-s", device.serial, *adb_args), cwd=WEBXR_DIR, timeout=timeout)

    def is_running(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def list_devices(self) -> Tuple[bool, List[AndroidDeviceInfo] | str]:
        ok, adb_or_message = self._ensure_adb()
        if not ok:
            return False, adb_or_message

        run_command((adb_or_message, "start-server"), cwd=WEBXR_DIR, timeout=15)
        ok, output = run_command((adb_or_message, "devices", "-l"), cwd=WEBXR_DIR, timeout=15)
        if not ok:
            return False, output

        devices: List[AndroidDeviceInfo] = []
        for line in output.splitlines():
            line = line.strip()
            if not line or line.startswith("List of devices attached") or line.startswith("* "):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            serial_id = parts[0]
            state = parts[1]
            extras: Dict[str, str] = {}
            for token in parts[2:]:
                if ":" not in token:
                    continue
                key, value = token.split(":", 1)
                extras[key] = value
            model = extras.get("model", "").replace("_", " ")
            detail = " ".join(parts[2:])
            devices.append(
                AndroidDeviceInfo(
                    serial=serial_id,
                    state=state,
                    model=model,
                    detail=detail,
                )
            )
        return True, devices

    def start(self, device: AndroidDeviceInfo) -> Tuple[bool, str]:
        self.stop()

        ok, gnirehtet_or_message = self._ensure_gnirehtet()
        if not ok:
            return False, gnirehtet_or_message

        creation_flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0) if os.name == "nt" else 0
        cwd = WEBXR_DIR / "platform-tools" if Path(gnirehtet_or_message).parent.name == "platform-tools" else WEBXR_DIR
        args = [gnirehtet_or_message, "run", device.serial]
        try:
            process = subprocess.Popen(
                args,
                cwd=str(cwd),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                shell=False,
                start_new_session=(os.name != "nt"),
                creationflags=creation_flags,
            )
        except Exception as exc:
            self._set_error(str(exc))
            return False, str(exc)

        with self._lock:
            self._process = process
            self._thread = threading.Thread(target=self._wait_process, daemon=True)
            self._last_error = ""
            self._device = device
            self._stop_requested = False
            thread = self._thread
        assert thread is not None
        thread.start()

        time.sleep(0.5)
        if process.poll() not in (None, 0):
            return False, self.status().last_error or "gnirehtet 启动失败。"
        return True, f"gnirehtet 已启动: {device.serial}"

    def force_stop_browser(self) -> Tuple[bool, str]:
        # ok, message = self._run_adb_for_device(("shell", "am", "force-stop", "com.oculus.browser"))
        # if ok:
        #     return True, "已执行 adb force-stop com.oculus.browser，Quest 浏览器 WebXR 环境已清理。"
        #
        # fallback_ok, _fallback_message = self._run_adb_for_device(("shell", "am", "force-stop", "com.meta.browser"))
        # if fallback_ok:
        #     return True, "已执行浏览器 force-stop（兼容包名 com.meta.browser）。"
        # return False, message
        return True, "已跳过 adb 关闭 Quest 浏览器指令。"

    def open_webxr_url(self, url: str) -> Tuple[bool, str]:
        if not url:
            return False, "没有可用于 Quest 的 WebXR 访问地址。"
        # ok, message = self._run_adb_for_device(
        #     (
        #         "shell",
        #         "am",
        #         "start",
        #         "-a",
        #         "android.intent.action.VIEW",
        #         "-n",
        #         "com.oculus.vrshell/.MainActivity",
        #         "-d",
        #         url,
        #     ),
        #     timeout=25.0,
        # )
        # if ok:
        #     return True, f"Quest 已尝试打开: {url}"
        # return False, message
        return True, f"已跳过 adb 启动 Quest 浏览器指令: {url}"

    def stop(self) -> None:
        with self._lock:
            process = self._process
            device = self._device
            self._stop_requested = True
            self._process = None
        if process is not None:
            stop_process_tree(process)

        ok, gnirehtet_or_message = self._ensure_gnirehtet()
        if ok and device is not None:
            run_command((gnirehtet_or_message, "stop", device.serial), cwd=WEBXR_DIR, timeout=20)

        with self._lock:
            self._device = None

    def _wait_process(self) -> None:
        with self._lock:
            process = self._process
        if process is None or process.stdout is None:
            return

        try:
            for _line in process.stdout:
                pass
        finally:
            code = process.poll()
            with self._lock:
                stop_requested = self._stop_requested
            if code not in (None, 0) and not stop_requested:
                self._set_error(f"gnirehtet 退出，返回码 {code}")

    def status(self) -> GnirehtetStatus:
        with self._lock:
            device = self._device
            return GnirehtetStatus(
                running=self._process is not None and self._process.poll() is None,
                device_serial=device.serial if device else "",
                device_model=device.model if device else "",
                last_error=self._last_error,
            )


def collect_cert_status() -> CertStatus:
    server_cert = CERT_DIR / "localhost.pem"
    server_key = CERT_DIR / "localhost-key.pem"
    root_cer = CERT_DIR / "quest-dev-root-ca.cer"
    root_pem = CERT_DIR / "quest-dev-root-ca.pem"
    https_ready = server_cert.exists() and server_key.exists()
    quest_ca_ready = root_cer.exists() and root_pem.exists()
    missing = [path for path in (server_cert, server_key, root_cer, root_pem) if not path.exists()]
    return CertStatus(
        https_ready=https_ready,
        quest_ca_ready=quest_ca_ready,
        server_cert=server_cert,
        server_key=server_key,
        root_cer=root_cer,
        root_pem=root_pem,
        missing_paths=missing,
    )


TITLE_TEXT = "CC-BRIDGE"
SUBTITLE_TEXT = "适用于CCtrl的WebXR桥接工具"
TITLE_GRADIENT = ("#7dd3fc", "#38bdf8", "#22c55e", "#f59e0b", "#f472b6")


def format_output_interface(output_if: int) -> str:
    return "USB" if int(output_if) == MB_OUTPUT_IF_USB else "RS232"


def format_xr_status(status: XrDeviceStatus) -> str:
    if not status.detected:
        return "等待 HELLO 握手"
    caps: List[str] = []
    if status.caps & XR_BRIDGE_CAP_PAYLOAD_96:
        caps.append("96B")
    if status.caps & XR_BRIDGE_CAP_DELTA_ARBITER:
        caps.append("ARB")
    if status.caps & XR_BRIDGE_CAP_CRC16:
        caps.append("CRC16")
    ready_text = "就绪" if status.bridge_ready else "待切 RS232"
    caps_text = "/".join(caps) if caps else "--"
    return (
        f"{ready_text} out={format_output_interface(status.output_if)} "
        f"seq={status.seq} age={status.age_ms}ms caps={caps_text}"
    )


def build_gradient_figlet(title: str) -> Text:
    figlet = Figlet(font="slant", width=160)
    rendered = figlet.renderText(title).rstrip("\n")
    glyph_count = sum(1 for char in rendered if char != " ")
    glyph_index = 0
    rich_text = Text()

    palette = list(TITLE_GRADIENT)
    palette_size = len(palette) - 1

    for line_no, line in enumerate(rendered.splitlines()):
        for char in line:
            if char == " ":
                rich_text.append(" ")
                continue
            ratio = 0.0 if glyph_count <= 1 else glyph_index / float(glyph_count - 1)
            slot = min(int(ratio * palette_size), palette_size)
            rich_text.append(char, style=f"bold {palette[slot]}")
            glyph_index += 1
        if line_no != len(rendered.splitlines()) - 1:
            rich_text.append("\n")
    return rich_text


def build_hero_renderable(bridge_host: str, bridge_port: int, http_port: int) -> Panel:
    title = Align.center(build_gradient_figlet(TITLE_TEXT))
    subtitle = Align.center(Text(SUBTITLE_TEXT, style="bold #dbeafe"))
    meta = Align.center(
        Text(
            f"Bridge {bridge_host}:{bridge_port}   |   WebXR {http_port}   |   {sys.platform}",
            style="#94a3b8",
        )
    )
    return Panel(
        Group(title, subtitle, meta),
        border_style="#2563eb",
        padding=(1, 2),
    )


def build_info_panel(title: str, rows: Sequence[Tuple[str, str]], border_style: str) -> Panel:
    table = Table.grid(padding=(0, 1))
    table.add_column(style="bold #e2e8f0", no_wrap=True, width=9)
    table.add_column(style="#cbd5e1", ratio=1)
    table.add_column(style="bold #e2e8f0", no_wrap=True, width=9)
    table.add_column(style="#cbd5e1", ratio=1)
    for index in range(0, len(rows), 2):
        left_label, left_value = rows[index]
        if index + 1 < len(rows):
            right_label, right_value = rows[index + 1]
        else:
            right_label, right_value = "", ""
        table.add_row(left_label, left_value, right_label, right_value)
    return Panel(table, title=title, border_style=border_style, padding=(0, 1))


def build_url_panel(urls: Sequence[str]) -> Panel:
    body = Text()
    if urls:
        for index, url in enumerate(urls):
            if index:
                body.append("\n")
            body.append("◆ ", style="bold #f59e0b")
            body.append(url, style="bold #e0f2fe")
    else:
        body.append("暂无可用地址", style="#94a3b8")
    return Panel(body, title="访问地址", border_style="#f59e0b", padding=(0, 1))


def build_runtime_panel(step_statuses: Sequence[Tuple[str, str]], current_step: str, busy: bool) -> Panel:
    icon_map = {
        "pending": ("○", "#64748b"),
        "working": ("◉", "#38bdf8"),
        "done": ("◆", "#22c55e"),
        "skip": ("◇", "#f59e0b"),
        "error": ("✕", "#ef4444"),
    }
    body = Text()
    body.append("当前状态: ", style="bold #e2e8f0")
    body.append(current_step, style="bold #7dd3fc" if busy else "bold #22c55e")
    body.append("\n")
    for index, (title, status) in enumerate(step_statuses):
        icon, color = icon_map.get(status, ("○", "#64748b"))
        body.append(icon + " ", style=f"bold {color}")
        body.append(title, style="#e5e7eb")
        if index != len(step_statuses) - 1:
            body.append("\n")
    return Panel(body, title="流程阶段", border_style="#8b5cf6", padding=(0, 1))


if UI_IMPORT_ERROR is None:

    class ChoiceScreen(ModalScreen[Optional[str]]):
        CSS = ""

        BINDINGS = [Binding("escape", "cancel", "取消")]

        def __init__(
            self,
            title: str,
            description: str,
            items: Sequence[Tuple[str, str]],
            *,
            confirm_label: str = "确认",
            skip_label: str = "跳过",
        ) -> None:
            super().__init__()
            self._title = title
            self._description = description
            self._items = list(items)
            self._confirm_label = confirm_label
            self._skip_label = skip_label

        def compose(self) -> ComposeResult:
            with Vertical(id="choice_dialog"):
                yield Label(self._title, id="choice_title")
                yield Static(self._description, id="choice_desc")
                yield OptionList(*[Option(label) for label, _value in self._items], id="choice_options")
                with Horizontal(id="choice_buttons"):
                    yield Button(self._confirm_label, id="choice_confirm", variant="primary")
                    yield Button(self._skip_label, id="choice_skip")

        def on_mount(self) -> None:
            option_list = self.query_one("#choice_options", OptionList)
            if self._items:
                option_list.highlighted = 0
            option_list.focus()

        def action_cancel(self) -> None:
            self.dismiss(None)

        def _selected_value(self) -> Optional[str]:
            option_list = self.query_one("#choice_options", OptionList)
            highlighted = option_list.highlighted
            if highlighted is None:
                return None
            return self._items[highlighted][1]

        def on_option_list_option_selected(self, _event: OptionList.OptionSelected) -> None:
            self.dismiss(self._selected_value())

        def on_button_pressed(self, event: Button.Pressed) -> None:
            if event.button.id == "choice_confirm":
                self.dismiss(self._selected_value())
            else:
                self.dismiss(None)


    class ConfirmScreen(ModalScreen[bool]):
        CSS = ""

        BINDINGS = [Binding("escape", "cancel", "取消")]

        def __init__(
            self,
            title: str,
            message: str,
            *,
            confirm_label: str = "确认",
            cancel_label: str = "跳过",
        ) -> None:
            super().__init__()
            self._title = title
            self._message = message
            self._confirm_label = confirm_label
            self._cancel_label = cancel_label

        def compose(self) -> ComposeResult:
            with Vertical(id="confirm_dialog"):
                yield Label(self._title, id="confirm_title")
                yield Static(self._message, id="confirm_message")
                with Horizontal(id="confirm_buttons"):
                    yield Button(self._confirm_label, id="confirm_yes", variant="primary")
                    yield Button(self._cancel_label, id="confirm_no")

        def action_cancel(self) -> None:
            self.dismiss(False)

        def on_button_pressed(self, event: Button.Pressed) -> None:
            self.dismiss(event.button.id == "confirm_yes")


    class CCBridgeTui(App[None]):
        CSS = ""

        BINDINGS = [
            Binding("q", "request_quit", "退出"),
            Binding("r", "refresh_now", "刷新"),
        ]

        def __init__(self, args: argparse.Namespace) -> None:
            super().__init__()
            runtime_config = load_runtime_config()
            self._baud = int(args.baud)
            self._preferred_serial_port = str(args.serial_port or "")
            self._preferred_android_serial = str(args.android_serial or "")
            self._bridge_host = str(runtime_config.get("bridgeHost", DEFAULT_BRIDGE_HOST) or DEFAULT_BRIDGE_HOST)
            self._bridge_port = int(runtime_config.get("bridgePort", DEFAULT_BRIDGE_PORT) or DEFAULT_BRIDGE_PORT)
            self._http_port = int(args.http_port)
            self._skip_cleanup = bool(args.skip_cleanup)

            self._bridge = XrUartBridgeManager()
            self._web_service = WebServiceManager(self._http_port)
            self._gnirehtet = GnirehtetManager()
            self._shutdown_started = False
            self._busy = False
            self._refreshing = False
            self._refresh_task: Optional[asyncio.Task[None]] = None
            self._last_status_poll_at = 0.0
            self._step_statuses: List[Tuple[str, str]] = []
            if not self._skip_cleanup:
                self._step_statuses.append(("残留进程清理", "pending"))
            self._step_statuses.extend(
                [
                    ("USB串口连接", "pending"),
                    ("gnirehtet连接", "pending"),
                    ("WebXR服务启动", "pending"),
                    ("证书检查和生成", "pending"),
                ]
            )
            self._current_step = "等待初始化"
            self._last_quest_launch_url = ""
            self._last_quest_launch_at = 0.0

        def compose(self) -> ComposeResult:
            yield Header(show_clock=True)
            with Vertical(id="root"):
                yield Static(build_hero_renderable(self._bridge_host, self._bridge_port, self._http_port), id="hero")
                with Horizontal(id="main"):
                    with Vertical(id="status_column"):
                        yield Label("状态总览", id="status_title")
                        yield Static(id="usb_card", classes="status_card")
                        yield Static(id="gnirehtet_card", classes="status_card")
                        yield Static(id="web_card", classes="status_card")
                        yield Static(id="cert_card", classes="status_card")
                        yield Static(id="url_card", classes="status_card")
                    with Vertical(id="log_column"):
                        yield Label("事件日志", id="log_title")
                        yield RichLog(id="event_log", markup=False, wrap=True, auto_scroll=True)
                    with Vertical(id="action_column"):
                        yield Label("初始化与控制", id="action_title")
                        yield LoadingIndicator(id="busy_indicator")
                        yield Static(id="step_status")
                        yield Button("校准XR坐标系: 关", id="action_calibration")
                        yield Button("重新选择USB串口", id="action_serial", variant="primary")
                        yield Button("重新连接gnirehtet", id="action_gnirehtet")
                        yield Button("重启WebXR服务", id="action_web")
                        yield Button("重新生成证书", id="action_cert")
                        yield Button("重新执行npm编译", id="action_build")
                        yield Button("退出", id="action_quit", variant="error")
            yield Footer()

        async def on_mount(self) -> None:
            self._busy_indicator = self.query_one("#busy_indicator", LoadingIndicator)
            self._busy_indicator.display = False
            self._sync_calibration_button()
            await self._refresh_dashboard(force_status_poll=False)
            self._refresh_task = asyncio.create_task(self._refresh_loop())
            self.run_worker(self._run_initial_sequence(), exclusive=True, group="ccbridge-init")

        async def on_unmount(self) -> None:
            if self._refresh_task is not None:
                self._refresh_task.cancel()

        async def _refresh_loop(self) -> None:
            try:
                while True:
                    await asyncio.sleep(1.0)
                    if not self._busy:
                        await self._refresh_dashboard(force_status_poll=True)
            except asyncio.CancelledError:
                return

        def _set_buttons_disabled(self, disabled: bool) -> None:
            for button_id in (
                "action_calibration",
                "action_serial",
                "action_gnirehtet",
                "action_web",
                "action_cert",
                "action_build",
                "action_quit",
            ):
                self.query_one(f"#{button_id}", Button).disabled = disabled

        def _set_busy(self, busy: bool, step: str = "") -> None:
            self._busy = busy
            self._busy_indicator.display = busy
            self._set_buttons_disabled(busy)
            if step:
                self._current_step = step
            self._update_runtime_panel()

        def _write_log(self, message: str, style: str = "#dbeafe") -> None:
            self.query_one("#event_log", RichLog).write(Text(message, style=style))

        def _log_phase(self, title: str, detail: str = "", *, color: str = "#38bdf8") -> None:
            marker = Text("◆ ", style=f"bold {color}")
            marker.append(title, style=f"bold {color}")
            if detail:
                marker.append("  ")
                marker.append(detail, style="#94a3b8")
            self.query_one("#event_log", RichLog).write(marker)

        def _update_step_state(self, title: str, state: str) -> None:
            for index, (step_title, _old_state) in enumerate(self._step_statuses):
                if step_title == title:
                    self._step_statuses[index] = (step_title, state)
                    break
            self._update_runtime_panel()

        def _update_runtime_panel(self) -> None:
            self.query_one("#step_status", Static).update(
                build_runtime_panel(self._step_statuses, self._current_step, self._busy)
            )

        def _sync_calibration_button(self) -> None:
            enabled = bool(load_runtime_config().get("calibrationModeEnabled", False))
            button = self.query_one("#action_calibration", Button)
            button.label = "校准XR坐标系: 开" if enabled else "校准XR坐标系: 关"
            button.variant = "primary" if enabled else "default"

        async def _run_blocking(self, step: str, func, *args):
            self._set_busy(True, step)
            try:
                return await asyncio.to_thread(func, *args)
            finally:
                self._set_busy(False)

        def _gather_dashboard(self) -> DashboardBundle:
            return DashboardBundle(
                bridge=self._bridge.snapshot(),
                gnirehtet=self._gnirehtet.status(),
                web=self._web_service.fetch_status(),
                cert=collect_cert_status(),
            )

        def _open_quest_webxr_sync(self) -> Tuple[bool, str]:
            web_status = self._web_service.fetch_status()
            if not web_status.running:
                return False, "WebXR 服务尚未运行，无法自动打开 Quest 页面。"
            url = pick_remote_access_url(web_status.access_urls)
            if not url:
                return False, "未找到可供 Quest 使用的非回环访问地址。"
            now = time.monotonic()
            if self._last_quest_launch_url == url and now - self._last_quest_launch_at < 2.0:
                return True, f"已跳过重复打开: {url}"
            ok, message = self._gnirehtet.open_webxr_url(url)
            if ok:
                self._last_quest_launch_url = url
                self._last_quest_launch_at = now
            return ok, message

        async def _maybe_open_quest_webxr(self, step_label: str) -> None:
            cert_status = collect_cert_status()
            if not cert_status.https_ready:
                self._write_log("HTTPS 证书尚未就绪，暂不自动打开 Quest WebXR 页面。", style="#fbbf24")
                return
            ok, message = await self._run_blocking("让 Quest 打开 WebXR 页面", self._open_quest_webxr_sync)
            self._write_log(message, style="#cbd5e1" if ok else "#fca5a5")
            self._log_phase(step_label, message, color="#22c55e" if ok else "#ef4444")

        async def _refresh_dashboard(self, *, force_status_poll: bool) -> None:
            if self._refreshing:
                return
            self._refreshing = True
            try:
                if force_status_poll and time.monotonic() - self._last_status_poll_at >= 2.5:
                    bridge_snapshot = self._bridge.snapshot()
                    if bridge_snapshot.running and bridge_snapshot.serial_connected:
                        await asyncio.to_thread(self._bridge.request_status, 0.45)
                        self._last_status_poll_at = time.monotonic()
                bundle = await asyncio.to_thread(self._gather_dashboard)
                self._apply_dashboard(bundle)
            finally:
                self._refreshing = False

        def _apply_dashboard(self, bundle: DashboardBundle) -> None:
            bridge = bundle.bridge
            bridge_rows = [
                ("USB", f"{bridge.serial_port or '--'} @ {bridge.serial_baud}" if bridge.running else "未连接"),
                ("串口桥", "在线" if bridge.serial_connected else "等待"),
                ("Web桥", "在线" if bridge.client_connected else "等待"),
                ("XR", format_xr_status(bridge.xr_status)),
            ]
            if bridge.last_error:
                bridge_rows.append(("错误", bridge.last_error))
            self.query_one("#usb_card", Static).update(build_info_panel("USB / XR桥接", bridge_rows, "#38bdf8"))

            gn = bundle.gnirehtet
            gn_rows = [
                ("状态", "运行中" if gn.running else "未连接"),
                ("设备", gn.device_model or gn.device_serial or "--"),
            ]
            if gn.last_error:
                gn_rows.append(("错误", gn.last_error))
            self.query_one("#gnirehtet_card", Static).update(build_info_panel("gnirehtet", gn_rows, "#22c55e"))

            web = bundle.web
            runtime_config = load_runtime_config()
            web_rows = [
                ("状态", "运行中" if web.running else "未运行"),
                ("客户端", f"{web.clients} clients / {web.active_sessions} sessions"),
                ("桥接", f"{web.bridge_host}:{web.bridge_port} {'OK' if web.bridge_connected else 'WAIT'}"),
                ("校准模式", "开启" if bool(runtime_config.get("calibrationModeEnabled", False)) else "关闭"),
            ]
            if web.last_error:
                web_rows.append(("错误", web.last_error))
            self.query_one("#web_card", Static).update(build_info_panel("WebXR 服务", web_rows, "#6366f1"))

            cert = bundle.cert
            cert_state = f"{'可用' if cert.https_ready else '缺失'} / {'可用' if cert.quest_ca_ready else '缺失'}"
            cert_rows = [
                ("HTTPS / Quest CA", cert_state),
                ("证书目录", relpath(CERT_DIR)),
            ]
            if cert.missing_paths:
                cert_rows.append(("缺失项", ", ".join(relpath(path) for path in cert.missing_paths)))
            self.query_one("#cert_card", Static).update(build_info_panel("证书", cert_rows, "#f59e0b"))
            self.query_one("#url_card", Static).update(build_url_panel(web.access_urls))
            self._sync_calibration_button()
            self._update_runtime_panel()

        async def _run_initial_sequence(self) -> None:
            self._log_phase("CC-BRIDGE 已启动", "准备执行初始化向导", color="#7dd3fc")
            if not self._skip_cleanup:
                await self._cleanup_step()
            await self._usb_step(step_label="USB串口连接")
            await self._gnirehtet_step(step_label="gnirehtet连接")
            await self._web_step(step_label="WebXR服务启动")
            await self._cert_step(step_label="证书检查和生成")
            self._current_step = "初始化完成"
            self._log_phase("初始化完成", "可以直接从右侧按钮执行后续操作", color="#22c55e")
            await self._refresh_dashboard(force_status_poll=False)

        async def _cleanup_step(self) -> None:
            title = "残留进程清理"
            self._update_step_state(title, "working")
            killed = await self._run_blocking(title, cleanup_residual_processes)
            if killed:
                self._log_phase(title, f"已关闭 {len(killed)} 个残留进程", color="#f59e0b")
                for entry in killed:
                    self._write_log(f"  {entry}", style="#94a3b8")
            else:
                self._log_phase(title, "未发现需要清理的残留进程", color="#22c55e")
            self._update_step_state(title, "done")

        async def _select_serial_port(self) -> Optional[SerialPortInfo]:
            ports = await asyncio.to_thread(list_serial_ports)
            if not ports:
                self._write_log("未扫描到可用串口。", style="#fbbf24")
                return None
            preferred = self._preferred_serial_port
            self._preferred_serial_port = ""
            if preferred:
                matched = next((port for port in ports if port.device == preferred), None)
                if matched is not None:
                    return matched
                self._write_log(f"预选串口 {preferred} 未找到，将转为手动选择。", style="#fbbf24")
            items = [(describe_serial_port(port), port.device) for port in ports]
            selected = await self.push_screen_wait(
                ChoiceScreen(
                    "选择 CCtrl USB 串口",
                    "请选择要连接的 USB 串口，按 Enter 确认，也可以直接双击或回车选中。",
                    items,
                    confirm_label="连接",
                    skip_label="跳过",
                )
            )
            if selected is None:
                return None
            return next((port for port in ports if port.device == selected), None)

        def _connect_serial_sync(self, port: SerialPortInfo) -> Tuple[bool, List[str]]:
            lines: List[str] = []
            ok, message = self._bridge.start(port.device, self._baud, self._bridge_host, self._bridge_port)
            if not ok:
                return False, [message]
            if not self._bridge.wait_for_serial_ready(2.0):
                snapshot = self._bridge.snapshot()
                self._bridge.stop()
                return False, [snapshot.last_error or "端口未就绪"]
            lines.append(f"串口已连接: {port.device} @ {self._baud}")
            status = self._bridge.request_status(timeout=1.0)
            if status is None:
                snapshot = self._bridge.snapshot()
                self._bridge.stop()
                return False, lines + [snapshot.last_error or "未收到设备 HELLO 应答"]
            lines.append(format_xr_status(status))
            if status.bridge_ready:
                lines.append("下位机已识别，XR 桥接可立即使用。")
            else:
                lines.append(
                    f"下位机已识别，但当前 LinkOut={format_output_interface(status.output_if)}。"
                )
                lines.append("切到 RS232 后会自动热恢复，无需重新连接。")
            return True, lines

        async def _usb_step(self, *, step_label: str) -> None:
            self._update_step_state(step_label, "working")
            if serial is None:
                self._log_phase(step_label, "缺少 pyserial，已跳过", color="#ef4444")
                self._update_step_state(step_label, "error")
                return
            while True:
                port = await self._select_serial_port()
                if port is None:
                    self._log_phase(step_label, "用户跳过 USB 串口连接", color="#f59e0b")
                    self._update_step_state(step_label, "skip")
                    return
                ok, lines = await self._run_blocking(step_label, self._connect_serial_sync, port)
                for line in lines:
                    self._write_log(line, style="#cbd5e1" if ok else "#fca5a5")
                if ok:
                    self._log_phase(step_label, f"{port.device} 串口桥已连接", color="#22c55e")
                    self._update_step_state(step_label, "done")
                    await self._refresh_dashboard(force_status_poll=False)
                    return
                retry = await self.push_screen_wait(
                    ConfirmScreen(
                        "USB 串口连接失败",
                        "\n".join(lines) + "\n\n是否重新选择串口？",
                        confirm_label="重试",
                        cancel_label="跳过",
                    )
                )
                if not retry:
                    self._log_phase(step_label, "用户在失败后选择跳过", color="#f59e0b")
                    self._update_step_state(step_label, "skip")
                    return

        async def _select_android_device(self) -> Optional[AndroidDeviceInfo]:
            ok, devices_or_message = await self._run_blocking("扫描 Android 设备", self._gnirehtet.list_devices)
            if not ok:
                self._write_log(str(devices_or_message), style="#fca5a5")
                return None
            devices = [item for item in devices_or_message if item.state == "device"]
            if not devices:
                self._write_log("当前没有处于 device 状态的 Android 设备。", style="#fbbf24")
                return None
            preferred = self._preferred_android_serial
            self._preferred_android_serial = ""
            if preferred:
                matched = next((device for device in devices if device.serial == preferred), None)
                if matched is not None:
                    return matched
                self._write_log(f"预选设备 {preferred} 未连接，将转为手动选择。", style="#fbbf24")
            items = [(describe_android_device(device), device.serial) for device in devices]
            selected = await self.push_screen_wait(
                ChoiceScreen(
                    "选择 Android 设备",
                    "选择要通过 adb + gnirehtet 提供网络的设备。",
                    items,
                    confirm_label="连接",
                    skip_label="跳过",
                )
            )
            if selected is None:
                return None
            return next((device for device in devices if device.serial == selected), None)

        async def _gnirehtet_step(self, *, step_label: str) -> None:
            self._update_step_state(step_label, "working")
            device = await self._select_android_device()
            if device is None:
                self._log_phase(step_label, "未选择 Android 设备，已跳过", color="#f59e0b")
                self._update_step_state(step_label, "skip")
                return
            ok, message = await self._run_blocking(step_label, self._gnirehtet.start, device)
            self._write_log(message, style="#cbd5e1" if ok else "#fca5a5")
            self._log_phase(step_label, message, color="#22c55e" if ok else "#ef4444")
            if ok:
                stop_ok, stop_message = await self._run_blocking(
                    "清理 Quest 浏览器 WebXR 环境",
                    self._gnirehtet.force_stop_browser,
                )
                self._write_log(stop_message, style="#cbd5e1" if stop_ok else "#fca5a5")
                self._log_phase(
                    "Quest 浏览器清理",
                    stop_message,
                    color="#22c55e" if stop_ok else "#ef4444",
                )
            self._update_step_state(step_label, "done" if ok else "error")
            await self._refresh_dashboard(force_status_poll=False)

        async def _web_step(self, *, step_label: str) -> None:
            self._update_step_state(step_label, "working")
            build_first = initial_build_needed()
            if build_first:
                self._write_log("检测到首次启动或 dist 缺失，将自动执行 npm run build。", style="#fbbf24")
            ok, message = await self._run_blocking(step_label, self._web_service.start_service, build_first)
            if build_first and ok:
                mark_initial_build_done()
            self._write_log(message, style="#cbd5e1" if ok else "#fca5a5")
            self._log_phase(step_label, message, color="#22c55e" if ok else "#ef4444")
            self._update_step_state(step_label, "done" if ok else "error")
            await self._refresh_dashboard(force_status_poll=False)
            if ok:
                await self._maybe_open_quest_webxr("Quest 自动打开")

        def _generate_certs_and_restart_sync(self) -> Tuple[bool, List[str]]:
            lines: List[str] = []
            ok, message = self._web_service.generate_certs()
            if not ok:
                return False, [message]
            lines.extend([line for line in message.splitlines() if line.strip()])
            if self._web_service.is_running():
                self._web_service.stop_service()
                ok, restart_message = self._web_service.start_service(build_first=False)
                lines.append(restart_message)
                if not ok:
                    return False, lines
            return True, lines

        async def _cert_step(self, *, step_label: str) -> None:
            self._update_step_state(step_label, "working")
            cert_status = collect_cert_status()
            if cert_status.https_ready and cert_status.quest_ca_ready:
                self._log_phase(step_label, "证书完整，HTTPS 可以直接使用", color="#22c55e")
                self._update_step_state(step_label, "done")
                await self._refresh_dashboard(force_status_poll=False)
                return
            confirm = await self.push_screen_wait(
                ConfirmScreen(
                    "生成 WebXR 证书",
                    "检测到证书不完整。\n\n是否现在生成或刷新本地 HTTPS 证书？",
                    confirm_label="生成",
                    cancel_label="跳过",
                )
            )
            if not confirm:
                self._log_phase(step_label, "用户跳过证书生成", color="#f59e0b")
                self._update_step_state(step_label, "skip")
                await self._refresh_dashboard(force_status_poll=False)
                return
            ok, lines = await self._run_blocking(step_label, self._generate_certs_and_restart_sync)
            for line in lines:
                self._write_log(line, style="#cbd5e1" if ok else "#fca5a5")
            self._log_phase(step_label, "证书生成流程完成" if ok else "证书生成失败", color="#22c55e" if ok else "#ef4444")
            self._update_step_state(step_label, "done" if ok else "error")
            await self._refresh_dashboard(force_status_poll=False)
            if ok:
                await self._maybe_open_quest_webxr("Quest 自动打开")

        async def _reselect_serial_action(self) -> None:
            if self._bridge.is_running():
                await self._run_blocking("关闭当前串口桥", self._bridge.stop)
            await self._usb_step(step_label="USB串口连接")

        async def _reconnect_gnirehtet_action(self) -> None:
            await self._run_blocking("停止当前 gnirehtet", self._gnirehtet.stop)
            await self._gnirehtet_step(step_label="gnirehtet连接")
            if self._web_service.is_running():
                await self._maybe_open_quest_webxr("Quest 自动打开")

        async def _restart_web_action(self) -> None:
            await self._run_blocking("停止 WebXR 服务", self._web_service.stop_service)
            ok, message = await self._run_blocking("重启 WebXR 服务", self._web_service.start_service, False)
            self._write_log(message, style="#cbd5e1" if ok else "#fca5a5")
            self._log_phase("WebXR服务重启", message, color="#22c55e" if ok else "#ef4444")
            await self._refresh_dashboard(force_status_poll=False)
            if ok:
                await self._maybe_open_quest_webxr("Quest 自动打开")

        def _set_calibration_mode_sync(self, enabled: bool) -> Tuple[bool, str]:
            try:
                config = load_runtime_config()
                config["calibrationModeEnabled"] = bool(enabled)
                save_runtime_config(config)
                return True, f"XR 坐标系校准模式已{'开启' if enabled else '关闭'}。"
            except Exception as exc:
                return False, f"写入运行配置失败: {exc}"

        async def _toggle_calibration_action(self) -> None:
            current = bool(load_runtime_config().get("calibrationModeEnabled", False))
            ok, message = await self._run_blocking(
                "切换 XR 坐标系校准模式",
                self._set_calibration_mode_sync,
                not current,
            )
            self._write_log(message, style="#cbd5e1" if ok else "#fca5a5")
            self._log_phase("校准XR坐标系", message, color="#22c55e" if ok else "#ef4444")
            await self._refresh_dashboard(force_status_poll=False)

        async def _regenerate_certs_action(self) -> None:
            ok, lines = await self._run_blocking("重新生成证书", self._generate_certs_and_restart_sync)
            for line in lines:
                self._write_log(line, style="#cbd5e1" if ok else "#fca5a5")
            self._log_phase("证书刷新", "完成" if ok else "失败", color="#22c55e" if ok else "#ef4444")
            await self._refresh_dashboard(force_status_poll=False)

        async def _rebuild_frontend_action(self) -> None:
            ok, message = await self._run_blocking("执行 npm run build", self._web_service.build_frontend)
            if ok:
                mark_initial_build_done()
            self._write_log(message, style="#cbd5e1" if ok else "#fca5a5")
            self._log_phase("npm 编译", "完成" if ok else "失败", color="#22c55e" if ok else "#ef4444")
            await self._refresh_dashboard(force_status_poll=False)

        async def on_button_pressed(self, event: Button.Pressed) -> None:
            if self._busy:
                return
            button_id = event.button.id
            if button_id == "action_calibration":
                self.run_worker(self._toggle_calibration_action(), exclusive=True, group="ccbridge-action")
            elif button_id == "action_serial":
                self.run_worker(self._reselect_serial_action(), exclusive=True, group="ccbridge-action")
            elif button_id == "action_gnirehtet":
                self.run_worker(self._reconnect_gnirehtet_action(), exclusive=True, group="ccbridge-action")
            elif button_id == "action_web":
                self.run_worker(self._restart_web_action(), exclusive=True, group="ccbridge-action")
            elif button_id == "action_cert":
                self.run_worker(self._regenerate_certs_action(), exclusive=True, group="ccbridge-action")
            elif button_id == "action_build":
                self.run_worker(self._rebuild_frontend_action(), exclusive=True, group="ccbridge-action")
            elif button_id == "action_quit":
                await self.action_request_quit()

        async def action_refresh_now(self) -> None:
            await self._refresh_dashboard(force_status_poll=True)
            self._log_phase("手动刷新", "面板状态已更新", color="#7dd3fc")

        async def action_request_quit(self) -> None:
            if self._busy:
                self._write_log("当前有任务在运行，请稍候后再退出。", style="#fbbf24")
                return
            await asyncio.to_thread(self.shutdown_services)
            self.exit()

        def shutdown_services(self) -> None:
            if self._shutdown_started:
                return
            self._shutdown_started = True
            self._web_service.stop_service()
            if self._bridge.is_running():
                self._bridge.stop()
            self._gnirehtet.stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CCtrl WebXR full-screen bridge CLI powered by Textual, Rich, and PyFiglet.",
    )
    parser.add_argument("--serial-port", default="", help="预选 USB 串口，如 COM6 或 /dev/ttyUSB0")
    parser.add_argument("--android-serial", default="", help="预选 adb 设备序列号")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="USB 串口波特率，默认 2000000")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help="WebXR 服务端口，默认 8787")
    parser.add_argument("--skip-cleanup", action="store_true", help="跳过启动前残留进程清理")
    parser.add_argument(
        "--ui-scale",
        type=float,
        default=None,
        help="TUI 缩放倍率，范围约 0.65~1.35；小终端可试 0.85",
    )
    parser.add_argument(
        "--ui-density",
        choices=TUI_DENSITY_VALUES,
        default=None,
        help="TUI 布局密度：compact / normal / comfortable",
    )
    return parser


def main() -> int:
    if UI_IMPORT_ERROR is not None:
        print(
            "Missing UI dependencies. Install them with: "
            "python -m pip install textual rich pyfiglet"
        )
        print(f"Import error: {UI_IMPORT_ERROR}")
        return 1

    parser = build_parser()
    args = parser.parse_args()
    if UI_IMPORT_ERROR is None:
        layout = resolve_tui_layout(args)
        ChoiceScreen.CSS = build_choice_screen_css(layout)
        ConfirmScreen.CSS = build_confirm_screen_css(layout)
        CCBridgeTui.CSS = build_app_css(layout)
    app = CCBridgeTui(args)
    try:
        app.run()
        return 0
    except KeyboardInterrupt:
        print("\n收到中断，准备退出。")
        return 130
    finally:
        app.shutdown_services()


if __name__ == "__main__":
    raise SystemExit(main())
