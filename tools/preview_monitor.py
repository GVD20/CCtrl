import argparse
import math
import struct
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import ttk
from typing import Deque, Dict, List, Optional, Tuple

try:
    import serial  # type: ignore
except ImportError:
    serial = None

RM_SOF = 0xA5
RM_CMD_ID = 0x0302
RM_DATA_LEN = 30
CRC8_INIT = 0xFF
CRC16_INIT = 0xFFFF

CRC8_TAB = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]

CRC16_TAB = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
]

ARM_L0_MM = 42.5
ARM_L1_MM = 100.0
ARM_L2_MM = 100.0


@dataclass(frozen=True)
class DecodedFrame:
    timestamp: float
    seq: int
    status: int
    err_flags: int
    key_flags: int
    delta_key: int
    wheel_pos: int
    joy_x: int
    joy_y: int
    pos: Tuple[float, float, float]
    euler: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]
    attitude_format: str
    output_if: str
    pose_mode: str
    mode_flags: int
    enc_raw: Tuple[int, int, int]
    reserved: int
    payload_hex: str


def crc8(data: bytes) -> int:
    value = CRC8_INIT
    for byte in data:
        value = CRC8_TAB[value ^ byte]
    return value


def crc16(data: bytes) -> int:
    value = CRC16_INIT
    for byte in data:
        value = ((value >> 8) ^ CRC16_TAB[(value ^ byte) & 0xFF]) & 0xFFFF
    return value


def half_to_float(half_word: int) -> float:
    packed = struct.pack("<H", half_word & 0xFFFF)
    try:
        return struct.unpack("<e", packed)[0]
    except struct.error:
        sign = -1.0 if (half_word & 0x8000) else 1.0
        exponent = (half_word >> 10) & 0x1F
        mantissa = half_word & 0x03FF
        if exponent == 0:
            if mantissa == 0:
                return sign * 0.0
            return sign * (mantissa / 1024.0) * (2.0 ** -14)
        if exponent == 0x1F:
            return float("inf") if mantissa == 0 else float("nan")
        return sign * (1.0 + mantissa / 1024.0) * (2.0 ** (exponent - 15))


class RmFrameParser:
    def __init__(self) -> None:
        self._buf = bytearray()
        self.total_frames = 0
        self.good_frames = 0
        self.bad_header_crc = 0
        self.bad_frame_crc = 0
        self.other_frames = 0

    def feed(self, chunk: bytes) -> List[DecodedFrame]:
        self._buf.extend(chunk)
        decoded = []

        while True:
            while self._buf and self._buf[0] != RM_SOF:
                del self._buf[0]

            if len(self._buf) < 5:
                break

            header = bytes(self._buf[:5])
            if crc8(header[:4]) != header[4]:
                self.bad_header_crc += 1
                del self._buf[0]
                continue

            data_len = header[1] | (header[2] << 8)
            frame_len = 5 + 2 + data_len + 2
            if len(self._buf) < frame_len:
                break

            frame = bytes(self._buf[:frame_len])
            del self._buf[:frame_len]
            self.total_frames += 1

            recv_crc16 = frame[-2] | (frame[-1] << 8)
            calc_crc16 = crc16(frame[:-2])
            if recv_crc16 != calc_crc16:
                self.bad_frame_crc += 1
                continue

            cmd_id = frame[5] | (frame[6] << 8)
            payload = frame[7:-2]
            if cmd_id != RM_CMD_ID or data_len != RM_DATA_LEN:
                self.other_frames += 1
                continue

            decoded.append(self._decode_payload(header[3], payload))
            self.good_frames += 1

        return decoded

    def stats(self) -> Dict[str, int]:
        return {
            "total": self.total_frames,
            "good": self.good_frames,
            "bad_header_crc": self.bad_header_crc,
            "bad_frame_crc": self.bad_frame_crc,
            "other": self.other_frames,
        }

    @staticmethod
    def _decode_payload(seq: int, payload: bytes) -> DecodedFrame:
        status_err = payload[0]
        key_flags = payload[1] | (payload[2] << 8)
        delta_key = payload[3]
        wheel_pos = struct.unpack_from("<h", payload, 4)[0]
        joy_x = payload[6]
        joy_y = payload[7]

        pos_h = struct.unpack_from("<3H", payload, 8)
        att_h = struct.unpack_from("<4H", payload, 14)
        enc_raw = struct.unpack_from("<3H", payload, 23)

        mode_flags = payload[22]
        output_if = "USB" if (mode_flags & 0x01) else "RS232"
        pose_mode = "ABS" if (mode_flags & 0x02) else "REL"
        attitude_format = "QUAT" if (mode_flags & 0x04) else "EUL"

        if attitude_format == "QUAT":
            quat = (
                half_to_float(att_h[0]),
                half_to_float(att_h[1]),
                half_to_float(att_h[2]),
                half_to_float(att_h[3]),
            )
            euler = (0.0, 0.0, 0.0)
        else:
            euler = (
                half_to_float(att_h[0]),
                half_to_float(att_h[1]),
                half_to_float(att_h[2]),
            )
            quat = (1.0, 0.0, 0.0, 0.0)

        return DecodedFrame(
            timestamp=time.time(),
            seq=seq,
            status=(status_err >> 4) & 0x0F,
            err_flags=status_err & 0x0F,
            key_flags=key_flags,
            delta_key=delta_key,
            wheel_pos=wheel_pos,
            joy_x=joy_x,
            joy_y=joy_y,
            pos=(half_to_float(pos_h[0]), half_to_float(pos_h[1]), half_to_float(pos_h[2])),
            euler=euler,
            quat=quat,
            attitude_format=attitude_format,
            output_if=output_if,
            pose_mode=pose_mode,
            mode_flags=mode_flags,
            enc_raw=(enc_raw[0], enc_raw[1], enc_raw[2]),
            reserved=payload[29],
            payload_hex=payload.hex(" "),
        )


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baud: int) -> None:
        super().__init__(daemon=True)
        self._port = port
        self._baud = baud
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self._parser = RmFrameParser()
        self._latest = None  # type: Optional[DecodedFrame]
        self._stats = self._parser.stats()
        self._times = deque(maxlen=300)  # type: Deque[float]
        self._connected = False
        self._error = ""

    def stop_worker(self) -> None:
        self._stop.set()
        self.join(timeout=1.0)

    def snapshot(self) -> Tuple[Optional[DecodedFrame], Dict[str, int], float, bool, str]:
        with self._lock:
            latest = self._latest
            stats = dict(self._stats)
            times = list(self._times)
            connected = self._connected
            error = self._error

        rate_hz = 0.0
        if len(times) >= 2:
            dt = times[-1] - times[0]
            if dt > 1e-6:
                rate_hz = (len(times) - 1) / dt

        return latest, stats, rate_hz, connected, error

    def run(self) -> None:
        if serial is None:
            with self._lock:
                self._error = "缺少 pyserial，请先执行: pip install pyserial"
            return

        ser = None
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.05)
            with self._lock:
                self._connected = True
                self._error = ""

            while not self._stop.is_set():
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue

                packets = self._parser.feed(chunk)
                if not packets:
                    with self._lock:
                        self._stats = self._parser.stats()
                    continue

                with self._lock:
                    for packet in packets:
                        self._latest = packet
                        self._times.append(packet.timestamp)
                    self._stats = self._parser.stats()
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            with self._lock:
                self._connected = False


class UnifiedPreviewMonitorApp(tk.Tk):
    def __init__(self, port: str, baud: int):
        super().__init__()
        self.title("编码器三维预览 + 30字节数据帧监视器")
        self.geometry("1320x780")
        self.minsize(1120, 700)

        self.port_var = tk.StringVar(value=port)
        self.baud_var = tk.StringVar(value=str(baud))
        self.status_var = tk.StringVar(value="未连接")
        self.rate_var = tk.StringVar(value="0.0 Hz")
        self.stats_var = tk.StringVar(value="有效帧=0 总帧=0 头CRC错=0 整帧CRC错=0 其他帧=0")

        self.worker = None  # type: Optional[SerialWorker]
        self.last_frame = None  # type: Optional[DecodedFrame]

        self.raw_enc = [0, 0, 0]
        self.ang_deg = [0.0, 0.0, 0.0]

        self.raw_vars = [tk.StringVar(value="0") for _ in range(3)]
        self.angle_vars = [tk.StringVar(value="0.00") for _ in range(3)]

        self.cam_yaw = -45.0
        self.cam_pitch = 30.0
        self.cam_scale = 2.2
        self.dragging = False
        self.last_mouse = (0, 0)

        self._build_ui()

        self.after(33, self._refresh)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)

        ttk.Label(top, text="串口端口").pack(side="left")
        ttk.Entry(top, textvariable=self.port_var, width=10).pack(side="left", padx=4)
        ttk.Label(top, text="波特率").pack(side="left")
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side="left", padx=4)
        ttk.Button(top, text="连接", command=self._connect).pack(side="left", padx=4)
        ttk.Button(top, text="断开", command=self._disconnect).pack(side="left", padx=4)

        ttk.Label(top, text="接收频率").pack(side="left", padx=(12, 2))
        ttk.Label(top, textvariable=self.rate_var).pack(side="left", padx=2)
        ttk.Label(top, textvariable=self.stats_var).pack(side="left", padx=10)
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=10)

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        tab_preview = ttk.Frame(notebook)
        tab_monitor = ttk.Frame(notebook)
        notebook.add(tab_preview, text="三维预览")
        notebook.add(tab_monitor, text="数据帧监视")

        left = ttk.Frame(tab_preview)
        left.pack(side="left", fill="y", padx=(0, 8), pady=6)
        right = ttk.Frame(tab_preview)
        right.pack(side="left", fill="both", expand=True, pady=6)

        ttk.Label(left, text="编码器实时预览 (Encoder Preview)", font=("Segoe UI", 11, "bold")).pack(anchor="w")
        ttk.Label(left, text="本页面为只读预览，不提供校准写入功能。", foreground="#555").pack(anchor="w", pady=(0, 8))

        for axis in range(3):
            box = ttk.LabelFrame(left, text="轴{}".format(axis + 1))
            box.pack(fill="x", pady=5)

            row0 = ttk.Frame(box)
            row0.pack(fill="x", padx=6, pady=4)
            ttk.Label(row0, text="编码器值(raw)").pack(side="left")
            ttk.Label(row0, textvariable=self.raw_vars[axis], width=8).pack(side="left", padx=4)
            ttk.Label(row0, text="角度(度)").pack(side="left", padx=(10, 0))
            ttk.Label(row0, textvariable=self.angle_vars[axis], width=8).pack(side="left", padx=4)

        self.canvas = tk.Canvas(right, width=820, height=680, bg="#11151a", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<ButtonPress-1>", self._on_mouse_down)
        self.canvas.bind("<B1-Motion>", self._on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_mouse_up)

        monitor_info = ttk.Frame(tab_monitor)
        monitor_info.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Label(monitor_info, text="30字节数据帧解析详情").pack(side="left")

        self.monitor_text = tk.Text(tab_monitor, font=("Consolas", 10), wrap="none")
        self.monitor_text.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.monitor_text.insert("1.0", "等待串口数据...\n")
        self.monitor_text.configure(state="disabled")

    def _connect(self) -> None:
        if serial is None:
            self.status_var.set("缺少 pyserial，请先执行: pip install pyserial")
            return

        self._disconnect()

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            self.status_var.set("波特率无效")
            return

        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("串口端口不能为空")
            return

        self.worker = SerialWorker(port, baud)
        self.worker.start()
        self.status_var.set("连接中: {} @ {}".format(port, baud))

    def _disconnect(self) -> None:
        if self.worker is not None:
            self.worker.stop_worker()
            self.worker = None
        self.status_var.set("未连接")

    def _on_mouse_down(self, event: tk.Event) -> None:
        self.dragging = True
        self.last_mouse = (event.x, event.y)

    def _on_mouse_drag(self, event: tk.Event) -> None:
        if not self.dragging:
            return
        dx = event.x - self.last_mouse[0]
        dy = event.y - self.last_mouse[1]
        self.last_mouse = (event.x, event.y)

        self.cam_yaw -= dx * 0.5
        self.cam_pitch += dy * 0.4
        if self.cam_pitch > 85.0:
            self.cam_pitch = 85.0
        if self.cam_pitch < -85.0:
            self.cam_pitch = -85.0

    def _on_mouse_up(self, _event: tk.Event) -> None:
        self.dragging = False

    @staticmethod
    def _angle_deg_from_calibrated(raw: int) -> float:
        signed = raw if raw <= 2048 else (raw - 4096)
        return (signed / 4096.0) * 360.0

    def _fk_points(self) -> List[Tuple[float, float, float]]:
        theta = [math.radians(self._angle_deg_from_calibrated(self.raw_enc[i])) for i in range(3)]
        t0, t1, t2 = theta
        t12 = t1 + t2

        p0 = (0.0, 0.0, 0.0)
        pa = (math.cos(t0) * ARM_L0_MM, 0.0, -math.sin(t0) * ARM_L0_MM)

        reach_b = ARM_L0_MM + ARM_L1_MM * math.cos(t1)
        py_b = ARM_L1_MM * math.sin(t1)
        pb = (math.cos(t0) * reach_b, py_b, -math.sin(t0) * reach_b)

        reach_c = ARM_L0_MM + ARM_L1_MM * math.cos(t1) + ARM_L2_MM * math.cos(t12)
        py_c = ARM_L1_MM * math.sin(t1) + ARM_L2_MM * math.sin(t12)
        pc = (math.cos(t0) * reach_c, py_c, -math.sin(t0) * reach_c)

        return [p0, pa, pb, pc]

    def _project(self, point: Tuple[float, float, float]) -> Tuple[float, float]:
        x, y, z = point
        yaw = math.radians(self.cam_yaw)
        pitch = math.radians(self.cam_pitch)

        x1 = x * math.cos(yaw) + y * math.sin(yaw)
        y1 = -x * math.sin(yaw) + y * math.cos(yaw)
        y2 = z * math.cos(pitch) - y1 * math.sin(pitch)

        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        cx, cy = width * 0.5, height * 0.58

        return (cx + x1 * self.cam_scale, cy - y2 * self.cam_scale)

    def _draw_preview(self) -> None:
        self.canvas.delete("all")

        grid_range = 220
        step = 20
        for value in range(-grid_range, grid_range + 1, step):
            p1 = self._project((value, -grid_range, 0.0))
            p2 = self._project((value, grid_range, 0.0))
            p3 = self._project((-grid_range, value, 0.0))
            p4 = self._project((grid_range, value, 0.0))

            color = "#2a313b" if value != 0 else "#49566b"
            self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=color, width=1)
            self.canvas.create_line(p3[0], p3[1], p4[0], p4[1], fill=color, width=1)

        origin = self._project((0.0, 0.0, 0.0))
        x_tip = self._project((120.0, 0.0, 0.0))
        y_tip = self._project((0.0, 120.0, 0.0))
        z_tip = self._project((0.0, 0.0, 120.0))

        self.canvas.create_line(origin[0], origin[1], x_tip[0], x_tip[1], fill="#f06565", width=2)
        self.canvas.create_line(origin[0], origin[1], y_tip[0], y_tip[1], fill="#63e6be", width=2)
        self.canvas.create_line(origin[0], origin[1], z_tip[0], z_tip[1], fill="#4dabf7", width=2)
        self.canvas.create_text(x_tip[0] + 10, x_tip[1], text="X", fill="#f06565")
        self.canvas.create_text(y_tip[0], y_tip[1] - 10, text="Y", fill="#63e6be")
        self.canvas.create_text(z_tip[0] + 10, z_tip[1], text="Z", fill="#4dabf7")

        points = self._fk_points()
        projected = [self._project(p) for p in points]

        for idx in range(3):
            x1, y1 = projected[idx]
            x2, y2 = projected[idx + 1]
            color = ["#b0b0b0", "#f6c453", "#74c0fc"][idx]
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=7)
            self.canvas.create_oval(x1 - 5, y1 - 5, x1 + 5, y1 + 5, fill=color, outline="")

        xe, ye = projected[-1]
        self.canvas.create_oval(xe - 7, ye - 7, xe + 7, ye + 7, fill="#8ce99a", outline="")
        self.canvas.create_text(12, 12, anchor="nw", fill="#cbd5e1", text="机械臂三维预览（基于校准后编码器值）")

    @staticmethod
    def _status_name(status: int) -> str:
        if status == 1:
            return "活动(ACTIVE)"
        if status == 2:
            return "节点断联(DISCONNECT)"
        return "空闲(IDLE)"

    @staticmethod
    def _err_flags_text(err_flags: int) -> str:
        names = []
        if err_flags & 0x01:
            names.append("CRC")
        if err_flags & 0x02:
            names.append("超时(TIMEOUT)")
        if err_flags & 0x04:
            names.append("解析(PARSE)")
        if err_flags & 0x08:
            names.append("拓扑(TOPO)")
        return "|".join(names) if names else "无(NONE)"

    @staticmethod
    def _keys_down_text(key_flags: int) -> str:
        pressed = ["KEY{}".format(i + 1) for i in range(8) if key_flags & (1 << i)]
        return " ".join(pressed) if pressed else "无(None)"

    def _monitor_dump(
        self,
        frame: Optional[DecodedFrame],
        stats: Dict[str, int],
        rate_hz: float,
        connected: bool,
        error: str,
    ) -> str:
        lines = []
        lines.append("统一30字节数据帧监视器 (RM 30-byte Monitor)")
        lines.append("=" * 86)

        lines.append("一、链路状态")
        lines.append("  连接状态(connected): {}".format("已连接" if connected else "未连接"))
        lines.append("  接收频率(rate): {:6.2f} Hz".format(rate_hz))
        lines.append(
            "  统计(stats): 有效(good)={good} 总帧(total)={total} 头CRC错(bad_header_crc)={bad_header_crc} 整帧CRC错(bad_frame_crc)={bad_frame_crc} 其他(other)={other}".format(
                **stats
            )
        )
        if error:
            lines.append("  工作线程错误(error): {}".format(error))

        lines.append("")

        if frame is None:
            lines.append("当前还没有收到有效帧。")
            return "\n".join(lines)

        lines.append("二、帧头与通道")
        lines.append("  时间戳(timestamp): {:.3f}".format(frame.timestamp))
        lines.append("  序号(seq): {}".format(frame.seq))
        lines.append("  输出接口(output_if): {}".format(frame.output_if))
        lines.append("  输出模式(pose_mode): {}".format(frame.pose_mode))
        lines.append("  姿态格式(attitude_format): {}".format("四元数(QUAT)" if frame.attitude_format == "QUAT" else "欧拉角(EUL)"))
        lines.append("  模式标志(mode_flags): 0x{:02X}".format(frame.mode_flags))

        lines.append("")
        lines.append("三、控制与输入")
        lines.append("  控制器状态(status): {}".format(self._status_name(frame.status)))
        lines.append("  错误标志(err_flags): 0x{:01X} ({})".format(frame.err_flags, self._err_flags_text(frame.err_flags)))
        lines.append("  按键位(key_flags): 0x{:04X} ({})".format(frame.key_flags, self._keys_down_text(frame.key_flags)))
        lines.append("  增量专用键(delta_key): {}".format(frame.delta_key))
        lines.append("  滚轮位置(wheel_pos): {}".format(frame.wheel_pos))
        lines.append("  摇杆(joy_x/joy_y): X={} Y={}".format(frame.joy_x, frame.joy_y))

        lines.append("")
        lines.append("四、运动数据")
        lines.append(
            "  位置(pos, mm): [{:+8.3f}, {:+8.3f}, {:+8.3f}]".format(
                frame.pos[0], frame.pos[1], frame.pos[2]
            )
        )
        if frame.attitude_format == "QUAT":
            lines.append(
                "  四元数(quat, wxyz): [{:+8.4f}, {:+8.4f}, {:+8.4f}, {:+8.4f}]".format(
                    frame.quat[0], frame.quat[1], frame.quat[2], frame.quat[3]
                )
            )
        else:
            lines.append(
                "  欧拉角(euler, deg): [{:+8.3f}, {:+8.3f}, {:+8.3f}]".format(
                    frame.euler[0], frame.euler[1], frame.euler[2]
                )
            )

        lines.append("")
        lines.append("五、编码器")
        lines.append(
            "  校准后编码器值(enc_raw): [{:4d}, {:4d}, {:4d}]".format(
                frame.enc_raw[0], frame.enc_raw[1], frame.enc_raw[2]
            )
        )
        lines.append("  保留字节(reserved): 0x{:02X}".format(frame.reserved))

        lines.append("")
        lines.append("六、原始载荷")
        lines.append("  payload[30] hex: {}".format(frame.payload_hex))
        return "\n".join(lines)

    def _refresh(self) -> None:
        if self.worker is not None:
            latest, stats, rate_hz, connected, error = self.worker.snapshot()
            if latest is not None:
                self.last_frame = latest
                self.raw_enc = [int(latest.enc_raw[0]), int(latest.enc_raw[1]), int(latest.enc_raw[2])]
                for i in range(3):
                    self.raw_vars[i].set(str(self.raw_enc[i]))

            self.rate_var.set("{:.1f} Hz".format(rate_hz))
            self.stats_var.set(
                "有效帧={} 总帧={} 头CRC错={} 整帧CRC错={} 其他帧={}".format(
                    stats.get("good", 0),
                    stats.get("total", 0),
                    stats.get("bad_header_crc", 0),
                    stats.get("bad_frame_crc", 0),
                    stats.get("other", 0),
                )
            )
            if error:
                self.status_var.set("错误: {}".format(error))
            elif connected:
                self.status_var.set("已连接")
            else:
                self.status_var.set("未连接")

            text = self._monitor_dump(self.last_frame, stats, rate_hz, connected, error)
            self.monitor_text.configure(state="normal")
            self.monitor_text.delete("1.0", "end")
            self.monitor_text.insert("1.0", text)
            self.monitor_text.configure(state="disabled")

        for i in range(3):
            self.ang_deg[i] = self._angle_deg_from_calibrated(self.raw_enc[i])
            self.angle_vars[i].set("{:.2f}".format(self.ang_deg[i]))

        self._draw_preview()
        self.after(33, self._refresh)

    def _on_close(self) -> None:
        self._disconnect()
        self.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="编码器三维预览 + 30字节数据帧监视器")
    parser.add_argument("--port", default="COM6", help="串口端口，例如 COM6")
    parser.add_argument("--baud", type=int, default=115200, help="串口波特率")
    args = parser.parse_args()

    app = UnifiedPreviewMonitorApp(args.port, args.baud)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
