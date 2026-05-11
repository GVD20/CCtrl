#!/usr/bin/env python3
from __future__ import annotations

import argparse
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

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:
    serial = None
    list_ports = None


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
PROCESS_SCAN_PATTERNS = (
    "webxr/server/index.mjs",
    "webxr/webxr_link.py",
    "tools/preview_monitor.py",
    "tools/serial_frame_debug.py",
    "tools/rs232_3d_viewer.py",
)


@dataclass
class XrDeviceStatus:
    mode: str = "NODE"
    requested: bool = False
    link_active: bool = False
    has_pose: bool = False
    restore_pending: bool = False
    seq: int = 0
    age_ms: int = 0
    raw_line: str = ""


@dataclass
class BridgeSnapshot:
    running: bool = False
    serial_port: str = ""
    serial_baud: int = DEFAULT_BAUD
    serial_connected: bool = False
    client_connected: bool = False
    bridge_host: str = DEFAULT_BRIDGE_HOST
    bridge_port: int = DEFAULT_BRIDGE_PORT
    frames_received: int = 0
    frames_forwarded: int = 0
    receive_rate_hz: float = 0.0
    forward_rate_hz: float = 0.0
    last_seq: int = 0
    last_packet_age_ms: int = 0
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
    transport: str = "stopped"
    serve_dist: bool = False
    clients: int = 0
    active_sessions: int = 0
    access_urls: List[str] = field(default_factory=list)
    bridge_connected: bool = False
    bridge_host: str = DEFAULT_BRIDGE_HOST
    bridge_port: int = DEFAULT_BRIDGE_PORT
    bridge_age_ms: int = 0
    bridge_last_seq: int = 0
    frames_received: int = 0
    frames_relayed: int = 0
    frames_dropped_seq: int = 0
    bridge_write_errors: int = 0
    receive_rate_hz: float = 0.0
    relay_rate_hz: float = 0.0
    latest_frame: Dict[str, object] = field(default_factory=dict)
    last_error: str = ""


@dataclass
class GnirehtetStatus:
    running: bool = False
    device_serial: str = ""
    device_model: str = ""
    last_error: str = ""
    last_log: str = ""


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
    }


def load_runtime_config() -> Dict[str, object]:
    config = default_runtime_config()
    if RUNTIME_CONFIG_PATH.exists():
        raw = load_json_file(RUNTIME_CONFIG_PATH)
        config.update(raw)
    return config


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
    return resolve_executable(
        (),
        ("node.exe", "node") if os.name == "nt" else ("node",),
    )


def resolve_npm_executable() -> Optional[str]:
    return resolve_executable(
        (),
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


def parse_xr_status_line(line: str) -> Optional[XrDeviceStatus]:
    text = line.strip()
    if not text.startswith("@XR STATUS "):
        return None

    values: Dict[str, str] = {}
    for part in text[len("@XR STATUS ") :].split():
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        values[key] = value

    return XrDeviceStatus(
        mode=values.get("mode", "NODE"),
        requested=values.get("requested", "0") == "1",
        link_active=values.get("link", "0") == "1",
        has_pose=values.get("has_pose", "0") == "1",
        restore_pending=values.get("restore", "0") == "1",
        seq=int(values.get("seq", "0") or 0),
        age_ms=int(values.get("age_ms", "0") or 0),
        raw_line=text,
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
        self._xr_line_events: Deque[Tuple[int, str]] = deque(maxlen=64)
        self._line_event_id = 0
        self._line_buffer = bytearray()
        self._packet_buffer = bytearray()
        self._packet_queue: Deque[Tuple[bytes, int]] = deque(maxlen=8)
        self._pending_packet = b""
        self._pending_seq = 0
        self._frames_received = 0
        self._frames_forwarded = 0
        self._last_seq = 0
        self._last_packet_at = 0.0
        self._rx_times: Deque[float] = deque(maxlen=300)
        self._tx_times: Deque[float] = deque(maxlen=300)

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
            self._packet_queue.clear()
            self._line_buffer.clear()
            self._pending_packet = b""
            self._pending_seq = 0

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

    def send_command(self, line: str) -> bool:
        payload = (line.rstrip("\r\n") + "\n").encode("ascii", errors="ignore")
        with self._lock:
            ser = self._serial
        if ser is None:
            return False
        try:
            ser.write(payload)
            ser.flush()
            return True
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._serial_connected = False
            return False

    def line_marker(self) -> int:
        with self._lock:
            return self._line_event_id

    def wait_for_line(
        self,
        predicate: Callable[[str], bool],
        timeout: float,
        *,
        after_id: Optional[int] = None,
    ) -> Optional[str]:
        deadline = time.monotonic() + timeout
        with self._condition:
            start_id = self._line_event_id if after_id is None else after_id
            while True:
                for event_id, line in self._xr_line_events:
                    if event_id > start_id and predicate(line):
                        return line
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)

    def request_status(self, timeout: float = 1.5) -> Optional[XrDeviceStatus]:
        marker = self.line_marker()
        if not self.send_command("@XR STATUS"):
            return None
        line = self.wait_for_line(
            lambda item: item.startswith("@XR STATUS "),
            timeout,
            after_id=marker,
        )
        if line is None:
            return None
        return parse_xr_status_line(line)

    def enter_xr_mode(self, timeout: float = 2.0) -> Tuple[bool, str]:
        marker = self.line_marker()
        if not self.send_command("@XR XR_ON"):
            return False, self.snapshot().last_error or "XR 指令发送失败。"
        line = self.wait_for_line(
            lambda item: item.startswith("@XR OK mode=UART") or "mode=UART" in item,
            timeout,
            after_id=marker,
        )
        if line is not None:
            return True, "下位机已进入 XR-UART 模式。"
        status = self.request_status(timeout=0.8)
        if status is not None and status.mode == "UART":
            return True, "下位机已进入 XR-UART 模式。"
        return False, self.snapshot().last_error or "等待 XR-UART 模式切换超时。"

    def exit_xr_mode(self, timeout: float = 3.0) -> Tuple[bool, str]:
        marker = self.line_marker()
        if not self.send_command("@XR XR_OFF"):
            return False, self.snapshot().last_error or "XR 退出指令发送失败。"
        line = self.wait_for_line(
            lambda item: item.startswith("@XR OK mode=NODE") or "mode=NODE" in item,
            timeout,
            after_id=marker,
        )
        if line is not None:
            return True, "下位机已退出 XR-UART 模式。"
        status = self.request_status(timeout=0.8)
        if status is not None and status.mode == "NODE":
            return True, "下位机已退出 XR-UART 模式。"
        return False, self.snapshot().last_error or "等待退出 XR-UART 模式超时。"

    def snapshot(self) -> BridgeSnapshot:
        with self._lock:
            age_ms = 0
            if self._last_packet_at > 0:
                age_ms = int(max(0.0, (time.time() - self._last_packet_at) * 1000.0))
            return BridgeSnapshot(
                running=self._running,
                serial_port=self._serial_port,
                serial_baud=self._serial_baud,
                serial_connected=self._serial_connected,
                client_connected=self._client_connected,
                bridge_host=self._bridge_host,
                bridge_port=self._bridge_port,
                frames_received=self._frames_received,
                frames_forwarded=self._frames_forwarded,
                receive_rate_hz=self._rate_hz(list(self._rx_times)),
                forward_rate_hz=self._rate_hz(list(self._tx_times)),
                last_seq=self._last_seq,
                last_packet_age_ms=age_ms,
                last_error=self._last_error,
                xr_status=XrDeviceStatus(**vars(self._xr_status)),
                xr_lines=list(self._xr_lines),
            )

    @staticmethod
    def _rate_hz(values: List[float]) -> float:
        if len(values) < 2:
            return 0.0
        dt = values[-1] - values[0]
        if dt <= 1e-6:
            return 0.0
        return (len(values) - 1) / dt

    def _record_xr_line(self, line: str) -> None:
        status = parse_xr_status_line(line)
        with self._condition:
            self._xr_lines.append(line)
            self._line_event_id += 1
            self._xr_line_events.append((self._line_event_id, line))
            if status is not None:
                self._xr_status = status
            self._condition.notify_all()

    def _consume_serial_side_channel(self, chunk: bytes) -> None:
        for byte in chunk:
            if not self._line_buffer:
                if byte == ord("@"):
                    self._line_buffer.append(byte)
                continue
            if byte == 13:
                continue
            if byte == 10:
                line = self._line_buffer.decode("ascii", errors="ignore")
                self._line_buffer.clear()
                if line.startswith("@XR "):
                    self._record_xr_line(line)
                continue
            if 32 <= byte <= 126:
                if len(self._line_buffer) < 180:
                    self._line_buffer.append(byte)
                else:
                    self._line_buffer.clear()
            else:
                self._line_buffer.clear()

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
            ser = serial.Serial(port, baud, timeout=0.01, write_timeout=0.5)
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

    def _close_serial(self) -> None:
        with self._lock:
            ser = self._serial
            self._serial = None
            self._serial_connected = False
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _forward_packet(self, packet: bytes, seq: int) -> None:
        with self._lock:
            ser = self._serial
        if ser is None:
            with self._lock:
                self._pending_packet = packet
                self._pending_seq = seq
            return

        try:
            ser.write(packet)
            ser.flush()
            now = time.time()
            with self._lock:
                self._frames_forwarded += 1
                self._tx_times.append(now)
                self._pending_packet = b""
                self._pending_seq = 0
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._pending_packet = packet
                self._pending_seq = seq
                self._serial_connected = False
            self._close_serial()

    def _flush_pending_packet(self) -> None:
        with self._lock:
            packet = self._pending_packet
            seq = self._pending_seq
            ready = self._serial is not None
        if ready and packet and seq:
            self._forward_packet(packet, seq)

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
            now = time.time()
            with self._lock:
                self._frames_received += 1
                self._last_seq = int(seq)
                self._last_packet_at = now
                self._rx_times.append(now)
                self._packet_queue.append((packet, int(seq)))

    def _tx_loop(self) -> None:
        target_interval = 1.0 / 60.0
        while not self._stop_event.is_set():
            start_time = time.perf_counter()
            packet = None
            seq = 0
            with self._lock:
                if self._packet_queue:
                    packet, seq = self._packet_queue[-1]
                    self._packet_queue.clear()
            if packet is not None:
                self._forward_packet(packet, seq)
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
                            self._consume_serial_side_channel(chunk)
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
        self._log_lines: Deque[str] = deque(maxlen=200)
        self._last_error = ""
        self._node_path = resolve_node_executable()
        self._npm_path = resolve_npm_executable()

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def _append_log(self, line: str) -> None:
        with self._lock:
            self._log_lines.append(line.rstrip())

    def recent_logs(self) -> List[str]:
        with self._lock:
            return list(self._log_lines)

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
            self._log_lines.clear()

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
            for line in process.stdout:
                self._append_log(line)
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
            status.transport = str(payload.get("transport", "unknown"))
            status.serve_dist = bool(payload.get("serveDist", False))
            status.clients = int(payload.get("clients", 0) or 0)
            status.active_sessions = int(payload.get("activeSessions", 0) or 0)

            relay = payload.get("relay", {})
            if isinstance(relay, dict):
                status.bridge_connected = bool(relay.get("bridgeConnected", False))
                status.bridge_host = str(relay.get("bridgeHost", status.bridge_host) or status.bridge_host)
                status.bridge_port = int(relay.get("bridgePort", status.bridge_port) or status.bridge_port)
                status.bridge_age_ms = int(relay.get("bridgeAgeMs", 0) or 0)
                status.bridge_last_seq = int(relay.get("bridgeLastSeq", 0) or 0)
                status.frames_received = int(relay.get("framesReceived", 0) or 0)
                status.frames_relayed = int(relay.get("framesRelayed", 0) or 0)
                status.frames_dropped_seq = int(relay.get("framesDroppedSeq", 0) or 0)
                status.bridge_write_errors = int(relay.get("bridgeWriteErrors", 0) or 0)
                status.receive_rate_hz = float(relay.get("receiveRateHz", 0.0) or 0.0)
                status.relay_rate_hz = float(relay.get("relayRateHz", 0.0) or 0.0)
                latest = relay.get("latestFrame", {})
                status.latest_frame = latest if isinstance(latest, dict) else {}
                status.last_error = str(relay.get("lastBridgeError", "") or status.last_error)

            urls = payload.get("accessUrls", [])
            if isinstance(urls, list) and urls:
                status.access_urls = [str(item) for item in urls if item]
            else:
                scheme = "https" if status.transport.startswith("https") else "http"
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
        self._log_lines: Deque[str] = deque(maxlen=120)
        self._last_error = ""
        self._device: Optional[AndroidDeviceInfo] = None
        self._stop_requested = False
        self._adb_path = resolve_adb_executable()
        self._gnirehtet_path = resolve_gnirehtet_executable()

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def _append_log(self, line: str) -> None:
        with self._lock:
            self._log_lines.append(line.rstrip())

    def _ensure_adb(self) -> Tuple[bool, str]:
        if self._adb_path:
            return True, self._adb_path
        return False, "未找到 adb。Windows 下请检查 webxr/platform-tools，Arch Linux 下请安装 adb 并加入 PATH。"

    def _ensure_gnirehtet(self) -> Tuple[bool, str]:
        if self._gnirehtet_path:
            return True, self._gnirehtet_path
        return False, "未找到 gnirehtet。Windows 下请检查 webxr/platform-tools，Arch Linux 下请安装 gnirehtet 并加入 PATH。"

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
            self._log_lines.clear()
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
            for line in process.stdout:
                self._append_log(line)
        finally:
            code = process.poll()
            with self._lock:
                stop_requested = self._stop_requested
            if code not in (None, 0) and not stop_requested:
                self._set_error(f"gnirehtet 退出，返回码 {code}")

    def status(self) -> GnirehtetStatus:
        with self._lock:
            device = self._device
            last_log = self._log_lines[-1] if self._log_lines else ""
            return GnirehtetStatus(
                running=self._process is not None and self._process.poll() is None,
                device_serial=device.serial if device else "",
                device_model=device.model if device else "",
                last_error=self._last_error,
                last_log=last_log,
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


class WebXRLinkApp:
    def __init__(self, args: argparse.Namespace) -> None:
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

    def run(self) -> None:
        print("WebXR Link CLI")
        print(f"Workspace: {REPO_ROOT}")
        print(f"Bridge: {self._bridge_host}:{self._bridge_port}    HTTP Port: {self._http_port}")
        print(f"Platform: {sys.platform}")

        if not self._skip_cleanup:
            print_step(0, 4, "残留进程清理")
            killed = cleanup_residual_processes()
            if killed:
                print_note("已关闭以下残留进程:")
                for entry in killed:
                    print(f"  - {entry}")
            else:
                print_note("未发现需要清理的残留进程。")

        self._initial_sequence()
        self._menu_loop()

    def _initial_sequence(self) -> None:
        self._setup_usb_serial(step_index=1, step_total=4)
        self._setup_gnirehtet(step_index=2, step_total=4)
        self._start_web_service(step_index=3, step_total=4)
        self._check_or_generate_certs(step_index=4, step_total=4)
        self._print_access_summary()

    def _setup_usb_serial(self, *, step_index: int, step_total: int) -> None:
        print_step(step_index, step_total, "USB串口连接")
        if serial is None:
            print_note("缺少 pyserial，已跳过 USB 串口步骤。")
            return

        preferred = self._preferred_serial_port
        self._preferred_serial_port = ""
        while True:
            port_info = choose_serial_port(preferred=preferred)
            preferred = ""
            if port_info is None:
                print_note("已跳过 USB 串口连接。")
                return

            ok, message = self._bridge.start(
                port_info.device,
                self._baud,
                self._bridge_host,
                self._bridge_port,
            )
            if not ok:
                print_note(f"串口桥启动失败: {message}")
                if not prompt_yes_no("是否重新选择串口?", True):
                    return
                continue

            if not self._bridge.wait_for_serial_ready(2.0):
                snapshot = self._bridge.snapshot()
                print_note(f"串口连接失败: {snapshot.last_error or '端口未就绪'}")
                self._bridge.stop()
                if not prompt_yes_no("是否重新选择串口?", True):
                    return
                continue

            print_note(f"串口已连接: {port_info.device} @ {self._baud}")
            status = self._bridge.request_status(timeout=1.0)
            if status is not None:
                print_note(self._format_xr_status(status))
            ok, message = self._bridge.enter_xr_mode(timeout=2.0)
            print_note(message)
            status = self._bridge.request_status(timeout=1.0)
            if ok and status is not None:
                print_note(self._format_xr_status(status))
            return

    def _setup_gnirehtet(self, *, step_index: int, step_total: int) -> None:
        print_step(step_index, step_total, "gnirehtet连接")
        ok, devices_or_message = self._gnirehtet.list_devices()
        if not ok:
            print_note(str(devices_or_message))
            print_note("已跳过 gnirehtet。")
            return

        devices = [item for item in devices_or_message if item.state == "device"]
        if not devices:
            print_note("当前没有处于 device 状态的 Android 设备，已跳过 gnirehtet。")
            return

        preferred = self._preferred_android_serial
        self._preferred_android_serial = ""
        selected: Optional[AndroidDeviceInfo] = None
        if preferred:
            selected = next((item for item in devices if item.serial == preferred), None)
            if selected is None:
                print_note(f"命令行指定设备 {preferred} 不在当前 adb 设备列表中，将转为手动选择。")

        if selected is None:
            print_note("可用 Android 设备:")
            for index, device in enumerate(devices, start=1):
                print(f"  {index}. {describe_android_device(device)}")
            print("  0. 跳过")
            while True:
                answer = input("请选择要连接 gnirehtet 的设备编号: ").strip()
                if answer == "0":
                    print_note("已跳过 gnirehtet。")
                    return
                if answer.isdigit():
                    index = int(answer)
                    if 1 <= index <= len(devices):
                        selected = devices[index - 1]
                        break
                print("请输入有效编号。")

        assert selected is not None
        ok, message = self._gnirehtet.start(selected)
        if ok:
            print_note(message)
        else:
            print_note(f"gnirehtet 启动失败: {message}")

    def _start_web_service(self, *, step_index: int, step_total: int) -> None:
        print_step(step_index, step_total, "WebXR服务启动")
        build_first = initial_build_needed()
        if build_first:
            print_note("检测到首次启动或 dist 缺失，自动执行 npm run build。")
        ok, message = self._web_service.start_service(build_first=build_first)
        if build_first and ok:
            mark_initial_build_done()
        if not ok:
            print_note(f"WebXR 服务启动失败: {message}")
            return
        print_note(message)
        health = self._web_service.fetch_status()
        if health.access_urls:
            print_note("当前访问地址:")
            for url in health.access_urls:
                print(f"  - {url}")

    def _check_or_generate_certs(self, *, step_index: int, step_total: int) -> None:
        print_step(step_index, step_total, "证书检查和生成")
        cert_status = collect_cert_status()
        self._print_cert_status(cert_status)
        if cert_status.https_ready and cert_status.quest_ca_ready:
            return

        if not prompt_yes_no("未检测到完整证书，是否现在生成?", True):
            print_note("已跳过证书生成。")
            return

        ok, message = self._web_service.generate_certs()
        if not ok:
            print_note(f"证书生成失败: {message}")
            return

        print_note("证书生成完成。")
        for line in message.splitlines():
            print(f"  {line}")

        if self._web_service.is_running():
            print_note("证书已更新，正在重启 WebXR 服务以启用 HTTPS。")
            self._web_service.stop_service()
            ok, restart_message = self._web_service.start_service(build_first=False)
            if ok:
                print_note(restart_message)
            else:
                print_note(f"WebXR 服务重启失败: {restart_message}")

    def _print_access_summary(self) -> None:
        health = self._web_service.fetch_status()
        urls = health.access_urls
        if not urls:
            return
        print("\nWebXR访问地址:")
        for url in urls:
            print(f"  - {url}")

    def _menu_loop(self) -> None:
        while True:
            self._print_dashboard()
            answer = input(
                "\n选择操作 [Enter刷新/1串口/2gnirehtet/3重启服务/4重建证书/5重新编译/6退出]: "
            ).strip()
            if answer == "":
                continue
            if answer == "1":
                self._reselect_serial()
                continue
            if answer == "2":
                self._reconnect_gnirehtet()
                continue
            if answer == "3":
                self._restart_web_service()
                continue
            if answer == "4":
                self._regenerate_certs()
                continue
            if answer == "5":
                self._rebuild_frontend()
                continue
            if answer == "6":
                return
            print("请输入有效选项。")

    def _refresh_xr_status(self) -> Optional[XrDeviceStatus]:
        snapshot = self._bridge.snapshot()
        if not snapshot.running or not snapshot.serial_connected:
            return snapshot.xr_status
        return self._bridge.request_status(timeout=0.7) or snapshot.xr_status

    def _print_dashboard(self) -> None:
        print("\n" + "=" * 72)
        self._refresh_xr_status()
        bridge = self._bridge.snapshot()
        gnirehtet_status = self._gnirehtet.status()
        web_status = self._web_service.fetch_status()
        cert_status = collect_cert_status()

        print("USB串口:")
        if bridge.running:
            print(
                f"  - {bridge.serial_port or '--'} @ {bridge.serial_baud}"
                f" | 串口={'OK' if bridge.serial_connected else 'WAIT'}"
                f" | Web桥={'OK' if bridge.client_connected else 'WAIT'}"
            )
            print(f"  - {self._format_xr_status(bridge.xr_status)}")
            print(
                f"  - RX {bridge.receive_rate_hz:.1f} Hz | TX {bridge.forward_rate_hz:.1f} Hz"
                f" | seq={bridge.last_seq} | age={bridge.last_packet_age_ms} ms"
            )
            if bridge.last_error:
                print(f"  - last_error: {bridge.last_error}")
        else:
            print("  - 未连接")

        print("gnirehtet:")
        if gnirehtet_status.running:
            label = gnirehtet_status.device_model or gnirehtet_status.device_serial
            print(f"  - 运行中 | {label}")
            if gnirehtet_status.last_log:
                print(f"  - {gnirehtet_status.last_log}")
        else:
            print("  - 未连接")
            if gnirehtet_status.last_error:
                print(f"  - last_error: {gnirehtet_status.last_error}")

        print("WebXR服务:")
        if web_status.running:
            print(
                f"  - 运行中 | transport={web_status.transport} | clients={web_status.clients}"
                f" | sessions={web_status.active_sessions} | bridge={'OK' if web_status.bridge_connected else 'WAIT'}"
            )
            print(
                f"  - RX {web_status.receive_rate_hz:.1f} Hz | Relay {web_status.relay_rate_hz:.1f} Hz"
                f" | seq={web_status.bridge_last_seq} | age={web_status.bridge_age_ms} ms"
            )
        else:
            print("  - 未运行")
        if web_status.last_error:
            print(f"  - last_error: {web_status.last_error}")

        print("证书:")
        self._print_cert_status(cert_status, prefix="  - ")

        print("WebXR访问地址:")
        if web_status.access_urls:
            for url in web_status.access_urls:
                print(f"  - {url}")
        else:
            print("  - 无可用地址")

    def _print_cert_status(self, cert_status: CertStatus, prefix: str = "  ") -> None:
        https_text = "HTTPS可用" if cert_status.https_ready else "HTTPS证书缺失"
        quest_text = "Quest导入证书可用" if cert_status.quest_ca_ready else "Quest导入证书缺失"
        print(f"{prefix}{https_text} | {quest_text}")
        print(f"{prefix}server cert: {relpath(cert_status.server_cert)}")
        print(f"{prefix}server key : {relpath(cert_status.server_key)}")
        print(f"{prefix}root cer   : {relpath(cert_status.root_cer)}")
        if cert_status.missing_paths:
            print(f"{prefix}missing    : {', '.join(relpath(path) for path in cert_status.missing_paths)}")

    def _format_xr_status(self, status: XrDeviceStatus) -> str:
        return (
            f"XR mode={status.mode} requested={int(status.requested)} "
            f"link={int(status.link_active)} pose={int(status.has_pose)} "
            f"restore={int(status.restore_pending)} seq={status.seq} age={status.age_ms} ms"
        )

    def _reselect_serial(self) -> None:
        if self._bridge.is_running():
            self._bridge.exit_xr_mode(timeout=1.5)
            self._bridge.stop()
        self._setup_usb_serial(step_index=1, step_total=1)

    def _reconnect_gnirehtet(self) -> None:
        self._gnirehtet.stop()
        self._setup_gnirehtet(step_index=1, step_total=1)

    def _restart_web_service(self) -> None:
        self._web_service.stop_service()
        ok, message = self._web_service.start_service(build_first=False)
        print_note(message if ok else f"WebXR 服务重启失败: {message}")
        if ok:
            self._print_access_summary()

    def _regenerate_certs(self) -> None:
        ok, message = self._web_service.generate_certs()
        if not ok:
            print_note(f"证书生成失败: {message}")
            return
        print_note("证书生成完成。")
        for line in message.splitlines():
            print(f"  {line}")
        if self._web_service.is_running():
            self._restart_web_service()

    def _rebuild_frontend(self) -> None:
        ok, message = self._web_service.build_frontend()
        if not ok:
            print_note(f"npm run build 失败: {message}")
            return
        mark_initial_build_done()
        print_note("npm run build 完成。")
        if self._web_service.is_running() and prompt_yes_no("是否立即重启 WebXR 服务以加载新 dist?", True):
            self._restart_web_service()

    def shutdown(self) -> None:
        if self._shutdown_started:
            return
        self._shutdown_started = True
        print("\n正在退出...")
        self._web_service.stop_service()
        if self._bridge.is_running():
            ok, message = self._bridge.exit_xr_mode(timeout=2.5)
            print_note(message if ok else f"退出 XR-UART 失败: {message}")
            self._bridge.stop()
        self._gnirehtet.stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CCtrl WebXR one-stop CLI: USB bridge, gnirehtet, certs, and WebXR service.",
    )
    parser.add_argument("--serial-port", default="", help="预选 USB 串口，如 COM6 或 /dev/ttyUSB0")
    parser.add_argument("--android-serial", default="", help="预选 adb 设备序列号")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="USB 串口波特率，默认 2000000")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help="WebXR 服务端口，默认 8787")
    parser.add_argument("--skip-cleanup", action="store_true", help="跳过启动前残留进程清理")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    app = WebXRLinkApp(args)
    try:
        app.run()
        return 0
    except KeyboardInterrupt:
        print("\n收到中断，准备退出。")
        return 130
    finally:
        app.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
