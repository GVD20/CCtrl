from __future__ import annotations

import argparse
import json
import math
import shutil
import socket
import ssl
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import mido
except ImportError:
    mido = None

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:
    serial = None
    list_ports = None

QT_IMPORT_ERROR: Optional[ImportError] = None
QFLUENT_IMPORT_ERROR: Optional[ImportError] = None

try:
    from PyQt5.QtCore import QPoint, QPointF, QRect, QRectF, QSize, Qt, QTimer, QUrl
    from PyQt5.QtGui import QColor, QDesktopServices, QFont, QIcon, QLinearGradient, QPainter, QPainterPath, QPen, QPixmap
    from PyQt5.QtWidgets import (
        QApplication,
        QAbstractItemView,
        QFileDialog,
        QFrame,
        QGridLayout,
        QHeaderView,
        QHBoxLayout,
        QLabel,
        QListWidget,
        QListWidgetItem,
        QScrollArea,
        QSizePolicy,
        QStyle,
        QStyledItemDelegate,
        QStyleOptionViewItem,
        QTableWidgetItem,
        QTreeWidgetItem,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    QT_IMPORT_ERROR = exc

if QT_IMPORT_ERROR is None:
    try:
        from qfluentwidgets import (
            BodyLabel,
            CaptionLabel,
            CardWidget,
            ComboBox,
            FluentWindow,
            FluentIcon as FIF,
            InfoBar,
            InfoBarIcon,
            InfoBarPosition,
            LineEdit,
            NavigationItemPosition,
            NavigationPushButton,
            NavigationWidget,
            ProgressBar,
            ProgressRing,
            PrimaryPushButton,
            PushButton,
            ListWidget,
            SpinBox,
            StrongBodyLabel,
            SubtitleLabel,
            TableWidget,
            TeachingTip,
            TeachingTipTailPosition,
            Theme,
            TitleLabel,
            TransparentToolButton,
            TreeWidget,
            isDarkTheme,
            qconfig,
            setFont,
            setTheme,
            setThemeColor,
        )
        from qfluentwidgets.components.widgets.info_bar import InfoIconWidget
    except ImportError as exc:
        QFLUENT_IMPORT_ERROR = exc

RM_SOF = 0xA5
RM_CMD_ID = 0x0302
RM_DATA_LEN = 30
CRC8_INIT = 0xFF
CRC16_INIT = 0xFFFF

ENCODER_MIN = 0
ENCODER_MAX = 4095
JOYSTICK_MIN = 0
JOYSTICK_MAX = 100
DEFAULT_WINDOW_SIZE = (1180, 730)
DEFAULT_MINIMUM_SIZE = (960, 420)
DEFAULT_THEME_COLOR = "#0F6CBD"
NAVIGATION_EXPANDED_WIDTH = 188
BAUD_OPTIONS = ("115200", "2000000")
KEY_DOT_COUNT = 7
PROJECT_GITHUB_URL = "https://github.com/GVD20/CCtrl"
WEBXR_DIR = Path(__file__).resolve().parents[1] / "webxr"
WEBXR_RUNTIME_CONFIG = WEBXR_DIR / "server" / "runtime-config.json"
WEBXR_LOCAL_HEALTH_URL = "https://127.0.0.1:8787/health"
WEBXR_LOCAL_STATUS_URL = "https://127.0.0.1:8787/status"
XR_UART_PACKET_SIZE = 96

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

MONITOR_TREE = [
    {
        "label": "链路",
        "description": "串口连接与校验状态",
        "children": [
            {"key": "serial.status", "label": "串口状态", "description": "友好化连接状态"},
            {"key": "serial.hint", "label": "状态说明", "description": "简洁说明信息"},
            {"key": "serial.rate", "label": "接收频率", "description": "最近有效帧接收频率"},
            {"key": "serial.crc_rate", "label": "CRC错误率", "description": "头CRC与帧CRC累计错误率"},
            {"key": "serial.good_frames", "label": "有效帧数", "description": "通过校验的有效帧"},
            {"key": "serial.total_frames", "label": "总帧数", "description": "接收到的总帧数"},
        ],
    },
    {
        "label": "帧信息",
        "description": "帧头和统一状态字节",
        "children": [
            {"key": "frame.timestamp", "label": "时间戳", "description": "最近有效帧时间"},
            {"key": "frame.seq", "label": "序号", "description": "帧序号"},
            {"key": "frame.status_err", "label": "statusErr源字节", "description": "高4位状态，低4位错误"},
            {"key": "frame.payload_hex", "label": "完整原始载荷", "description": "30字节载荷十六进制"},
        ],
    },
    {
        "label": "模式与状态",
        "description": "控制器状态和输出模式",
        "children": [
            {"key": "status.controller", "label": "控制器状态", "description": "解析后的控制器状态"},
            {"key": "status.err_flags", "label": "错误标志", "description": "解析后的错误标志"},
            {"key": "mode.output_if", "label": "输出接口", "description": "USB 或 RS232"},
            {"key": "mode.pose_mode", "label": "位姿模式", "description": "ABS 或 REL"},
            {"key": "mode.attitude_format", "label": "姿态格式", "description": "EUL 或 QUAT"},
            {"key": "mode.flags", "label": "modeFlags源字节", "description": "模式组合位"},
        ],
    },
    {
        "label": "按键与输入",
        "description": "按键、滚轮与摇杆数据",
        "children": [
            {"key": "input.key_flags", "label": "keyFlags源值", "description": "所有按键位源值"},
            {"key": "input.key1", "label": "KEY1", "description": "解析后的按键位"},
            {"key": "input.key2", "label": "KEY2", "description": "解析后的按键位"},
            {"key": "input.key3", "label": "KEY3", "description": "解析后的按键位"},
            {"key": "input.key4", "label": "KEY4", "description": "解析后的按键位"},
            {"key": "input.key5", "label": "KEY5", "description": "解析后的按键位"},
            {"key": "input.key6", "label": "KEY6", "description": "解析后的按键位"},
            {"key": "input.key7", "label": "KEY7", "description": "解析后的按键位"},
            {"key": "input.key8", "label": "KEY8", "description": "解析后的按键位"},
            {"key": "input.delta_key", "label": "deltaKey", "description": "增量基准按键"},
            {"key": "input.wheel_pos", "label": "wheelPos", "description": "滚轮累计值"},
            {"key": "input.joy_x", "label": "joyX", "description": "摇杆X"},
            {"key": "input.joy_y", "label": "joyY", "description": "摇杆Y"},
        ],
    },
    {
        "label": "运动数据",
        "description": "位置和姿态输出",
        "children": [
            {"key": "motion.pos_x", "label": "位置 X", "description": "毫米"},
            {"key": "motion.pos_y", "label": "位置 Y", "description": "毫米"},
            {"key": "motion.pos_z", "label": "位置 Z", "description": "毫米"},
            {"key": "motion.pos_half", "label": "位置源半精度", "description": "payload[8:13]"},
            {"key": "motion.euler_roll", "label": "Euler Roll", "description": "度"},
            {"key": "motion.euler_pitch", "label": "Euler Pitch", "description": "度"},
            {"key": "motion.euler_yaw", "label": "Euler Yaw", "description": "度"},
            {"key": "motion.quat_w", "label": "Quat W", "description": "四元数"},
            {"key": "motion.quat_x", "label": "Quat X", "description": "四元数"},
            {"key": "motion.quat_y", "label": "Quat Y", "description": "四元数"},
            {"key": "motion.quat_z", "label": "Quat Z", "description": "四元数"},
            {"key": "motion.att_half", "label": "姿态源半精度", "description": "payload[14:21]"},
        ],
    },
    {
        "label": "编码器",
        "description": "编码器值与导出角度",
        "children": [
            {"key": "encoder.group", "label": "encRaw源组", "description": "payload[23:28]"},
            {"key": "encoder.1.raw", "label": "编码器1 原始值", "description": "0..4095"},
            {"key": "encoder.1.deg", "label": "编码器1 角度", "description": "校准后角度"},
            {"key": "encoder.2.raw", "label": "编码器2 原始值", "description": "0..4095"},
            {"key": "encoder.2.deg", "label": "编码器2 角度", "description": "校准后角度"},
            {"key": "encoder.3.raw", "label": "编码器3 原始值", "description": "0..4095"},
            {"key": "encoder.3.deg", "label": "编码器3 角度", "description": "校准后角度"},
        ],
    },
    {
        "label": "载荷字段",
        "description": "按 payload 分段查看源数据",
        "children": [
            {"key": "payload.byte0", "label": "payload[0] statusErr", "description": "统一状态字节"},
            {"key": "payload.bytes1_2", "label": "payload[1:2] keyFlags", "description": "按键源字"},
            {"key": "payload.byte3", "label": "payload[3] deltaKey", "description": "deltaKey 源字节"},
            {"key": "payload.bytes4_5", "label": "payload[4:5] wheelPos", "description": "滚轮源字节"},
            {"key": "payload.byte6", "label": "payload[6] joyX", "description": "摇杆X源字节"},
            {"key": "payload.byte7", "label": "payload[7] joyY", "description": "摇杆Y源字节"},
            {"key": "payload.bytes8_13", "label": "payload[8:13] posH", "description": "位置半精度原始值"},
            {"key": "payload.bytes14_21", "label": "payload[14:21] attH", "description": "姿态半精度原始值"},
            {"key": "payload.byte22", "label": "payload[22] modeFlags", "description": "模式源字节"},
            {"key": "payload.bytes23_28", "label": "payload[23:28] encRaw", "description": "编码器源字组"},
            {"key": "payload.byte29", "label": "payload[29] reserved", "description": "保留字节"},
        ],
    },
]

FIELD_META: Dict[str, Tuple[str, str, str]] = {}


def _register_field_meta(nodes: Iterable[dict], parents: Tuple[str, ...] = ()) -> None:
    for node in nodes:
        label = node["label"]
        description = node["description"]
        children = node.get("children")
        if children:
            _register_field_meta(children, parents + (label,))
            continue
        key = node["key"]
        category = " / ".join(parents) if parents else "其它"
        FIELD_META[key] = (category, label, description)


_register_field_meta(MONITOR_TREE)


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
class RelayHealth:
    running: bool = False
    transport: str = "stopped"
    serve_dist: bool = False
    clients: int = 0
    active_sessions: int = 0
    bridge_host: str = "127.0.0.1"
    bridge_port: int = 8791
    bridge_connected: bool = False
    bridge_age_ms: int = 0
    bridge_last_seq: int = 0
    position_scale: float = 1.0
    rotation_scale: float = 1.0
    access_urls: List[str] = field(default_factory=list)
    frames_received: int = 0
    frames_relayed: int = 0
    frames_dropped_seq: int = 0
    bridge_write_errors: int = 0
    receive_rate_hz: float = 0.0
    relay_rate_hz: float = 0.0
    latest_frame: Dict[str, object] = field(default_factory=dict)
    last_error: str = ""


@dataclass(frozen=True)
class DecodedFrame:
    timestamp: float
    seq: int
    status: int
    err_flags: int
    status_err_byte: int
    key_flags: int
    delta_key: int
    wheel_pos: int
    joy_x: int
    joy_y: int
    pos: Tuple[float, float, float]
    pos_half_raw: Tuple[int, int, int]
    euler: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]
    att_half_raw: Tuple[int, int, int, int]
    attitude_format: str
    output_if: str
    pose_mode: str
    mode_flags: int
    enc_raw: Tuple[int, int, int]
    reserved: int
    payload_bytes: bytes
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


def calibrated_raw_to_deg(raw: int) -> float:
    signed = raw if raw <= 2048 else (raw - 4096)
    return (signed / 4096.0) * 360.0


def controller_status_name(status: int) -> str:
    if status == 1:
        return "活动"
    if status == 2:
        return "断联"
    return "空闲"


def err_flags_text(err_flags: int) -> str:
    names = []
    if err_flags & 0x01:
        names.append("CRC")
    if err_flags & 0x02:
        names.append("超时")
    if err_flags & 0x04:
        names.append("解析")
    if err_flags & 0x08:
        names.append("拓扑")
    return " | ".join(names) if names else "无"


def web_key_flags_text(key_flags: int) -> str:
    labels = [f"KEY{index + 1}" for index in range(KEY_DOT_COUNT) if key_flags & (1 << index)]
    return " + ".join(labels) if labels else "NONE"


def payload_slice_hex(payload: bytes, start: int, end: int) -> str:
    return payload[start:end].hex(" ") if payload else "--"


def crc_error_rate(stats: Dict[str, int]) -> float:
    total = stats.get("total", 0)
    if total <= 0:
        return 0.0
    errors = stats.get("bad_header_crc", 0) + stats.get("bad_frame_crc", 0)
    return float(errors) * 100.0 / float(total)


def summarize_serial_error(message: str) -> Tuple[str, str, str]:
    if not message:
        return ("neutral", "未连接", "选择串口并开始监视")

    lower = message.lower()
    if "access is denied" in lower or "permission" in lower:
        return ("error", "串口被占用", "关闭其他进程后重试")
    if "could not open port" in lower or "filenotfounderror" in lower or "no such file" in lower:
        return ("error", "串口不可用", "确认串口名称和设备连接")
    if "clearcommerror" in lower or "device not functioning" in lower:
        return ("error", "串口通信失败", "检查线缆、电源和设备状态")
    if "timed out" in lower:
        return ("error", "串口响应超时", "设备没有在预期时间内返回数据")
    if "pyserial" in lower:
        return ("error", "缺少依赖", "请先安装 pyserial")
    return ("error", "连接失败", "检查串口权限、端口占用和设备状态")


def link_status_summary(connected: bool, error: str) -> Tuple[str, str, str]:
    if error:
        return summarize_serial_error(error)
    if connected:
        return ("success", "已连接", "正在接收数据")
    return ("neutral", "未连接", "选择串口并开始监视")


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


def default_runtime_config() -> Dict[str, object]:
    return {
        "bridgeHost": "127.0.0.1",
        "bridgePort": 8791,
        "positionScale": 1.0,
        "rotationScale": 1.0,
    }


def load_runtime_config() -> Dict[str, object]:
    config = default_runtime_config()
    try:
        if WEBXR_RUNTIME_CONFIG.exists():
            data = json.loads(WEBXR_RUNTIME_CONFIG.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                config.update(data)
    except Exception:
        pass
    return config


def save_runtime_config(config: Dict[str, object]) -> None:
    merged = default_runtime_config()
    merged.update(config)
    WEBXR_RUNTIME_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    WEBXR_RUNTIME_CONFIG.write_text(
        json.dumps(merged, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def list_access_urls(port: int = 8787) -> List[str]:
    urls: List[str] = []
    seen: set[str] = set()
    hostname = socket.gethostname()
    candidates = ["127.0.0.1", "localhost"]
    try:
        for _family, _type, _proto, _canon, sockaddr in socket.getaddrinfo(
            hostname, None, socket.AF_INET
        ):
            ip = sockaddr[0]
            if ip not in candidates:
                candidates.append(ip)
    except socket.gaierror:
        pass

    for host in candidates:
        if host in seen:
            continue
        seen.add(host)
        urls.append(f"https://{host}:{port}")
    return urls


def list_serial_port_names() -> List[str]:
    if list_ports is None:
        return []
    return [info.device for info in list_ports.comports()]


def fetch_local_json(url: str) -> Dict[str, object]:
    request = urllib.request.Request(
        url, headers={"User-Agent": "CCtrlPreviewMonitor/1.0"}
    )
    context = ssl._create_unverified_context()
    with urllib.request.urlopen(request, timeout=1.5, context=context) as response:
        payload = response.read().decode("utf-8")
    data = json.loads(payload)
    return data if isinstance(data, dict) else {}


def post_local_json(url: str, payload: Dict[str, object]) -> Dict[str, object]:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        headers={
            "Content-Type": "application/json; charset=utf-8",
            "User-Agent": "CCtrlPreviewMonitor/1.0",
        },
        method="POST",
    )
    context = ssl._create_unverified_context()
    with urllib.request.urlopen(request, timeout=2.0, context=context) as response:
        payload_text = response.read().decode("utf-8")
    data = json.loads(payload_text)
    return data if isinstance(data, dict) else {}


def resolve_node_executable() -> Optional[str]:
    candidates = [
        shutil.which("node.exe"),
        shutil.which("node"),
        str(Path("C:/Program Files/nodejs/node.exe")),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).exists():
            return str(Path(candidate))
    return None


class WebServiceManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen[str]] = None
        self._reader: Optional[threading.Thread] = None
        self._log_lines: Deque[str] = deque(maxlen=120)
        self._last_error = ""
        self._node_path = resolve_node_executable()

    def _ensure_node(self) -> Tuple[bool, str]:
        if self._node_path and Path(self._node_path).exists():
            return True, self._node_path
        message = "未找到 node.exe，请先安装 Node.js 或修复 PATH"
        self._set_error(message)
        return False, message

    def _node_args(self, *parts: object) -> Tuple[bool, List[str] | str]:
        ok, node_or_message = self._ensure_node()
        if not ok:
            return False, str(node_or_message)
        args = [str(node_or_message)]
        for part in parts:
            args.append(str(part))
        return True, args

    def is_running(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def recent_logs(self) -> List[str]:
        with self._lock:
            return list(self._log_lines)

    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def _append_log(self, line: str) -> None:
        with self._lock:
            self._log_lines.append(line.rstrip())

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def run_command(self, args: Sequence[str]) -> Tuple[bool, str]:
        try:
            completed = subprocess.run(
                list(args),
                cwd=str(WEBXR_DIR),
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                shell=False,
                timeout=180,
                check=False,
            )
        except Exception as exc:
            self._set_error(str(exc))
            return False, str(exc)

        output = (completed.stdout or "") + (completed.stderr or "")
        for line in output.splitlines():
            self._append_log(line)
        if completed.returncode != 0:
            message = output.strip() or f"Command failed: {' '.join(args)}"
            self._set_error(message)
            return False, message
        return True, output.strip() or "OK"

    def generate_certs(self) -> Tuple[bool, str]:
        ok, args = self._node_args(WEBXR_DIR / "scripts" / "generate-dev-certs.mjs")
        if not ok:
            return False, str(args)
        return self.run_command(args)

    def build_frontend(self) -> Tuple[bool, str]:
        steps = [
            [WEBXR_DIR / "node_modules" / "typescript" / "bin" / "tsc", "--noEmit"],
            [WEBXR_DIR / "node_modules" / "vite" / "bin" / "vite.js", "build"],
        ]
        final_message = "OK"
        for step in steps:
            ok, args = self._node_args(*step)
            if not ok:
                return False, str(args)
            ok, final_message = self.run_command(args)
            if not ok:
                return False, final_message
        return True, final_message

    def start_service(self, build_first: bool = True) -> Tuple[bool, str]:
        if self.is_running():
            return True, "Web service already running"

        if build_first:
            ok, message = self.build_frontend()
            if not ok:
                return False, message

        try:
            ok, args = self._node_args(WEBXR_DIR / "server" / "index.mjs", "--serve-dist")
            if not ok:
                return False, str(args)
            process = subprocess.Popen(
                args,
                cwd=str(WEBXR_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                shell=False,
            )
        except Exception as exc:
            self._set_error(str(exc))
            return False, str(exc)

        with self._lock:
            self._process = process
            self._last_error = ""
            self._log_lines.clear()

        self._reader = threading.Thread(target=self._read_stdout, daemon=True)
        self._reader.start()
        deadline = time.time() + 6.0
        while time.time() < deadline:
            if not self.is_running():
                return False, self.last_error() or "Web service exited unexpectedly"
            try:
                fetch_local_json(WEBXR_LOCAL_STATUS_URL)
                return True, "Web service started"
            except Exception:
                time.sleep(0.15)
        return False, self.last_error() or "Web service start timed out"

    def stop_service(self) -> None:
        with self._lock:
            process = self._process
            self._process = None
        if process is None:
            return
        try:
            process.terminate()
            process.wait(timeout=3)
        except Exception:
            try:
                process.kill()
            except Exception:
                pass

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
                self._set_error(f"Web service exited with code {code}")

    def fetch_health(self) -> RelayHealth:
        status = RelayHealth(
            running=self.is_running(),
            access_urls=list_access_urls(),
            last_error=self.last_error(),
        )
        config = load_runtime_config()
        status.bridge_host = str(config.get("bridgeHost", "127.0.0.1") or "127.0.0.1")
        status.bridge_port = int(config.get("bridgePort", 8791) or 8791)
        status.position_scale = float(config.get("positionScale", 1.0) or 1.0)
        status.rotation_scale = float(config.get("rotationScale", 1.0) or 1.0)
        if not status.running:
            return status

        try:
            health = fetch_local_json(WEBXR_LOCAL_STATUS_URL)
            status.transport = str(health.get("transport", "https+wss"))
            status.serve_dist = bool(health.get("serveDist", False))
            status.clients = int(health.get("clients", 0) or 0)
            status.active_sessions = int(health.get("activeSessions", 0) or 0)
            relay = health.get("relay", {})
            if isinstance(relay, dict):
                status.bridge_host = str(relay.get("bridgeHost", status.bridge_host) or status.bridge_host)
                status.bridge_port = int(relay.get("bridgePort", status.bridge_port) or status.bridge_port)
                status.bridge_connected = bool(relay.get("bridgeConnected", False))
                status.bridge_age_ms = int(relay.get("bridgeAgeMs", 0) or 0)
                status.bridge_last_seq = int(relay.get("bridgeLastSeq", 0) or 0)
                status.position_scale = float(relay.get("positionScale", status.position_scale) or status.position_scale)
                status.rotation_scale = float(relay.get("rotationScale", status.rotation_scale) or status.rotation_scale)
                status.frames_received = int(relay.get("framesReceived", 0) or 0)
                status.frames_relayed = int(relay.get("framesRelayed", 0) or 0)
                status.frames_dropped_seq = int(relay.get("framesDroppedSeq", 0) or 0)
                status.bridge_write_errors = int(relay.get("bridgeWriteErrors", 0) or 0)
                status.receive_rate_hz = float(relay.get("receiveRateHz", 0.0) or 0.0)
                status.relay_rate_hz = float(relay.get("relayRateHz", 0.0) or 0.0)
                latest_frame = relay.get("latestFrame", {})
                status.latest_frame = latest_frame if isinstance(latest_frame, dict) else {}
                status.last_error = str(relay.get("lastBridgeError", "") or status.last_error)
            urls = health.get("accessUrls", [])
            if isinstance(urls, list):
                status.access_urls = [str(item) for item in urls if item]
        except Exception as exc:
            status.last_error = str(exc)
        return status


MELODY_JSON_VERSION = 1
NOTE_NAME_TABLE = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
QT_USER_ROLE_BASE = Qt.UserRole if QT_IMPORT_ERROR is None else 0x0100
TRACK_ID_ROLE = QT_USER_ROLE_BASE
TRACK_TITLE_ROLE = QT_USER_ROLE_BASE + 1
TRACK_SUMMARY_ROLE = QT_USER_ROLE_BASE + 2
TRACK_EMOJI_ROLE = QT_USER_ROLE_BASE + 3


@dataclass
class MelodyNote:
    start_beats: float
    duration_beats: float
    midi_note: int
    source_order: int = 0
    channel: int = 0

    @property
    def end_beats(self) -> float:
        return self.start_beats + self.duration_beats


@dataclass
class MelodyTrack:
    track_id: str
    name: str
    emoji: str
    notes: List[MelodyNote] = field(default_factory=list)
    source_notes: List[MelodyNote] = field(default_factory=list)
    note_total: int = 0
    chord_count: int = 0
    duration_ms: float = 0.0
    source_name: str = ""
    program: Optional[int] = None
    channel: Optional[int] = None
    is_json_module: bool = False


@dataclass
class MelodyPwmStep:
    freq_hz: int
    duration_ms: int


@dataclass
class MelodyUploadPackage:
    name: str
    bpm: int
    gap_ms: int
    steps: List[MelodyPwmStep]

    @property
    def total_ms(self) -> int:
        return sum(step.duration_ms for step in self.steps)


def midi_note_to_freq_hz(note: int) -> int:
    return int(round(440.0 * (2.0 ** ((note - 69) / 12.0))))


def midi_note_name(note: int) -> str:
    octave = note // 12 - 1
    return f"{NOTE_NAME_TABLE[note % 12]}{octave}"


def beats_to_ms(beats: float, bpm: int) -> float:
    bpm = max(1, bpm)
    return float(beats) * 60000.0 / float(bpm)


def format_ms_compact(milliseconds: float) -> str:
    ms = max(0.0, float(milliseconds))
    total_seconds = int(round(ms / 1000.0))
    minutes, seconds = divmod(total_seconds, 60)
    hours, minutes = divmod(minutes, 60)
    if hours > 0:
        return f"{hours}:{minutes:02d}:{seconds:02d}"
    return f"{minutes}:{seconds:02d}"


def format_note_time(milliseconds: float) -> str:
    ms = max(0.0, float(milliseconds))
    if ms >= 1000.0:
        return f"{ms / 1000.0:.3f} s"
    return f"{ms:.0f} ms"


def melody_track_summary(track: MelodyTrack) -> str:
    return f"{track.note_total} 音符 / {track.chord_count} 和弦 · {format_ms_compact(track.duration_ms)}"


def _count_chords(notes: Sequence[MelodyNote]) -> int:
    groups: Dict[int, int] = {}
    for note in notes:
        key = int(round(note.start_beats * 1000000.0))
        groups[key] = groups.get(key, 0) + 1
    return sum(1 for count in groups.values() if count > 1)


def _refresh_track_counters(track: MelodyTrack) -> None:
    track.note_total = len(track.notes)
    track.chord_count = _count_chords(track.notes)
    if not track.notes:
        track.duration_ms = 0.0


def _choose_track_emoji(name: str, program: Optional[int], is_drum: bool) -> str:
    lower = name.lower()
    if is_drum or any(key in lower for key in ("drum", "perc", "kit", "鼓")):
        return "🥁"
    if any(key in lower for key in ("piano", "keys", "keyboard", "ep", "clav", "钢琴")):
        return "🎹"
    if any(key in lower for key in ("guitar", "gtr", "ukulele", "banjo", "吉他")):
        return "🎸"
    if any(key in lower for key in ("bass", "贝斯")):
        return "🎸"
    if any(key in lower for key in ("string", "violin", "viola", "cello", "弦", "提琴")):
        return "🎻"
    if any(key in lower for key in ("trumpet", "trombone", "horn", "brass", "号")):
        return "🎺"
    if any(key in lower for key in ("sax", "clarinet", "oboe", "flute", "woodwind", "wind", "笛")):
        return "🎷"
    if any(key in lower for key in ("vocal", "voice", "choir", "lead vox", "人声")):
        return "🎤"
    if any(key in lower for key in ("synth", "pad", "lead", "arp")):
        return "🎛️"

    if program is None:
        return "🎼"
    if 0 <= program <= 7:
        return "🎹"
    if 24 <= program <= 39:
        return "🎸"
    if 40 <= program <= 47:
        return "🎻"
    if 56 <= program <= 63:
        return "🎺"
    if 64 <= program <= 79:
        return "🎷"
    if 112 <= program <= 119:
        return "🥁"
    return "🎼"


def _build_tempo_map(mid) -> Tuple[List[int], List[int], List[float]]:
    if mido is None:
        return ([0], [500000], [0.0])

    ticks = [0]
    tempos = [500000]
    elapsed_seconds = [0.0]
    absolute_tick = 0
    current_tempo = 500000
    current_seconds = 0.0

    for msg in mido.merge_tracks(mid.tracks):
        if msg.time:
            current_seconds += mido.tick2second(msg.time, mid.ticks_per_beat, current_tempo)
            absolute_tick += msg.time

        if msg.type == "set_tempo":
            if absolute_tick == ticks[-1]:
                tempos[-1] = msg.tempo
                elapsed_seconds[-1] = current_seconds
            else:
                ticks.append(absolute_tick)
                tempos.append(msg.tempo)
                elapsed_seconds.append(current_seconds)
            current_tempo = msg.tempo

    return ticks, tempos, elapsed_seconds


def _tick_to_ms(absolute_tick: int, ticks_per_beat: int, tempo_ticks: Sequence[int], tempos: Sequence[int], elapsed_seconds: Sequence[float]) -> float:
    if mido is None:
        return 0.0

    index = max(0, bisect_right(tempo_ticks, absolute_tick) - 1)
    base_tick = tempo_ticks[index]
    base_tempo = tempos[index]
    base_seconds = elapsed_seconds[index]
    tail_seconds = mido.tick2second(absolute_tick - base_tick, ticks_per_beat, base_tempo)
    return (base_seconds + tail_seconds) * 1000.0


def _resolve_melody_overlaps(notes: Sequence[MelodyNote]) -> List[MelodyNote]:
    resolved: List[MelodyNote] = []
    for note in sorted(notes, key=lambda item: (item.start_beats, item.source_order, item.midi_note)):
        current = MelodyNote(
            start_beats=note.start_beats,
            duration_beats=note.duration_beats,
            midi_note=note.midi_note,
            source_order=note.source_order,
            channel=note.channel,
        )

        while resolved and current.start_beats < resolved[-1].end_beats - 1e-9:
            previous = resolved[-1]
            new_duration = current.start_beats - previous.start_beats
            if new_duration <= 1e-9:
                resolved.pop()
                continue
            previous.duration_beats = new_duration
            break

        if current.duration_beats > 1e-9:
            resolved.append(current)

    return resolved


def _add_pwm_step(steps: List[MelodyPwmStep], freq_hz: int, duration_ms_float: float) -> None:
    duration_ms = int(round(duration_ms_float))
    if duration_ms <= 0:
        return

    if steps and steps[-1].freq_hz == freq_hz:
        steps[-1].duration_ms += duration_ms
    else:
        steps.append(MelodyPwmStep(freq_hz=freq_hz, duration_ms=duration_ms))


def build_upload_package(track: MelodyTrack, bpm: int, gap_ms: int) -> MelodyUploadPackage:
    bpm = max(1, int(bpm))
    gap_ms = max(0, int(gap_ms))
    steps: List[MelodyPwmStep] = []
    cursor_ms = 0.0

    for note in sorted(track.notes, key=lambda item: (item.start_beats, item.source_order, item.midi_note)):
        start_ms = beats_to_ms(note.start_beats, bpm)
        end_ms = beats_to_ms(note.end_beats, bpm)
        start_ms = max(start_ms, cursor_ms)
        end_ms = max(end_ms, start_ms)

        if start_ms > cursor_ms:
            _add_pwm_step(steps, 0, start_ms - cursor_ms)

        note_duration_ms = end_ms - start_ms
        if note_duration_ms <= 0.0:
            cursor_ms = end_ms
            continue

        play_ms = max(0.0, note_duration_ms - float(gap_ms))
        tail_rest_ms = note_duration_ms - play_ms
        if play_ms > 0.0:
            _add_pwm_step(steps, midi_note_to_freq_hz(note.midi_note), play_ms)
        if tail_rest_ms > 0.0:
            _add_pwm_step(steps, 0, tail_rest_ms)

        cursor_ms = end_ms

    while steps and steps[-1].freq_hz == 0:
        steps.pop()

    return MelodyUploadPackage(name=track.name, bpm=bpm, gap_ms=gap_ms, steps=steps)


def melody_module_document(track: MelodyTrack, bpm: int, gap_ms: int) -> Dict[str, object]:
    package = build_upload_package(track, bpm, gap_ms)
    note_count = len(track.notes)
    chord_count = _count_chords(track.notes)
    return {
        "version": MELODY_JSON_VERSION,
        "kind": "cctrl.melody.module",
        "source": {
            "name": track.source_name,
            "track_id": track.track_id,
            "is_json_module": track.is_json_module,
        },
        "track": {
            "name": track.name,
            "emoji": track.emoji,
            "note_count": note_count,
            "chord_count": chord_count,
            "duration_ms": package.total_ms,
            "program": track.program,
            "channel": track.channel,
        },
        "bpm": int(bpm),
        "gap_ms": int(gap_ms),
        "notes": [
            {
                "start_beats": round(note.start_beats, 6),
                "duration_beats": round(note.duration_beats, 6),
                "midi_note": int(note.midi_note),
                "channel": int(note.channel),
            }
            for note in track.notes
        ],
        "upload_preview": {
            "buzzer_step_struct": "BuzzerPwmStep",
            "step_count": len(package.steps),
            "total_ms": package.total_ms,
            "steps": [
                {"freq_hz": step.freq_hz, "duration_ms": step.duration_ms}
                for step in package.steps
            ],
        },
    }


def load_melody_module(path: Path) -> Tuple[MelodyTrack, int, int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("JSON 音频格式无效")

    bpm = int(data.get("bpm", 120))
    gap_ms = int(data.get("gap_ms", 30))
    track_meta = data.get("track", {})
    if not isinstance(track_meta, dict):
        track_meta = {}

    notes_value = data.get("notes", [])
    if not isinstance(notes_value, list):
        raise ValueError("JSON 音频缺少 notes 列表")

    notes: List[MelodyNote] = []
    for index, item in enumerate(notes_value):
        if not isinstance(item, dict):
            continue

        if "start_beats" in item:
            start_beats = float(item.get("start_beats", 0.0))
        else:
            start_beats = float(item.get("start_ms", 0.0)) * float(bpm) / 60000.0

        if "duration_beats" in item:
            duration_beats = float(item.get("duration_beats", 0.0))
        else:
            duration_beats = float(item.get("duration_ms", 0.0)) * float(bpm) / 60000.0

        midi_note = int(item.get("midi_note", item.get("note", 60)))
        notes.append(
            MelodyNote(
                start_beats=start_beats,
                duration_beats=max(0.0, duration_beats),
                midi_note=midi_note,
                source_order=index,
                channel=int(item.get("channel", 0)),
            )
        )

    notes = _resolve_melody_overlaps(notes)
    duration_beats = max((note.end_beats for note in notes), default=0.0)
    duration_ms = beats_to_ms(duration_beats, bpm)
    name = str(track_meta.get("name") or data.get("name") or path.stem)
    program_value = track_meta.get("program")
    channel_value = track_meta.get("channel")
    program = int(program_value) if program_value is not None else None
    channel = int(channel_value) if channel_value is not None else None

    track = MelodyTrack(
        track_id="json:module",
        name=name,
        emoji=str(track_meta.get("emoji") or _choose_track_emoji(name, program, channel == 9)),
        notes=notes,
        source_notes=list(notes),
        note_total=len(notes),
        chord_count=_count_chords(notes),
        duration_ms=duration_ms,
        source_name=path.name,
        program=program,
        channel=channel,
        is_json_module=True,
    )
    return track, bpm, gap_ms


def extract_melody_tracks(midi_path: Path) -> Tuple[List[MelodyTrack], int]:
    if mido is None:
        raise RuntimeError("请先安装 mido")

    mid = mido.MidiFile(midi_path)
    tempo_ticks, tempos, elapsed_seconds = _build_tempo_map(mid)
    bpm = int(round(60000000.0 / float(tempos[0]))) if tempos else 120
    bpm = max(20, min(300, bpm))

    tracks: List[MelodyTrack] = []
    ticks_per_beat = mid.ticks_per_beat

    for track_index, track in enumerate(mid.tracks):
        absolute_tick = 0
        note_order = 0
        track_name = ""
        instrument_name = ""
        active_notes: Dict[Tuple[int, int], Tuple[int, int]] = {}
        channel_counts: Dict[int, int] = {}
        program_by_channel: Dict[int, int] = {}
        raw_notes: List[MelodyNote] = []

        for msg in track:
            absolute_tick += msg.time

            if msg.type == "track_name" and getattr(msg, "name", "").strip() and not track_name:
                track_name = msg.name.strip()
                continue
            if msg.type == "instrument_name" and getattr(msg, "name", "").strip() and not instrument_name:
                instrument_name = msg.name.strip()
                continue
            if msg.type == "program_change":
                program_by_channel[msg.channel] = msg.program
                continue

            if msg.type == "note_on" and getattr(msg, "velocity", 0) > 0:
                channel = int(getattr(msg, "channel", 0))
                key = (channel, int(msg.note))
                channel_counts[channel] = channel_counts.get(channel, 0) + 1
                if key in active_notes:
                    start_tick, order = active_notes.pop(key)
                    if absolute_tick > start_tick:
                        raw_notes.append(
                            MelodyNote(
                                start_beats=start_tick / float(ticks_per_beat),
                                duration_beats=(absolute_tick - start_tick) / float(ticks_per_beat),
                                midi_note=int(msg.note),
                                source_order=order,
                                channel=channel,
                            )
                        )
                active_notes[key] = (absolute_tick, note_order)
                note_order += 1
                continue

            if msg.type == "note_off" or (msg.type == "note_on" and getattr(msg, "velocity", 0) == 0):
                channel = int(getattr(msg, "channel", 0))
                key = (channel, int(msg.note))
                active = active_notes.pop(key, None)
                if active is not None:
                    start_tick, order = active
                    if absolute_tick > start_tick:
                        raw_notes.append(
                            MelodyNote(
                                start_beats=start_tick / float(ticks_per_beat),
                                duration_beats=(absolute_tick - start_tick) / float(ticks_per_beat),
                                midi_note=int(msg.note),
                                source_order=order,
                                channel=channel,
                            )
                        )

        for (channel, midi_note), (start_tick, order) in active_notes.items():
            if absolute_tick > start_tick:
                raw_notes.append(
                    MelodyNote(
                        start_beats=start_tick / float(ticks_per_beat),
                        duration_beats=(absolute_tick - start_tick) / float(ticks_per_beat),
                        midi_note=midi_note,
                        source_order=order,
                        channel=channel,
                    )
                )

        if not raw_notes:
            continue

        raw_notes.sort(key=lambda item: (item.start_beats, item.source_order, item.midi_note))
        main_channel = max(channel_counts, key=channel_counts.get) if channel_counts else 0
        program = program_by_channel.get(main_channel)
        name = instrument_name or track_name or f"轨道 {track_index + 1}"
        end_tick = int(round(max(note.end_beats for note in raw_notes) * float(ticks_per_beat)))
        duration_ms = _tick_to_ms(end_tick, ticks_per_beat, tempo_ticks, tempos, elapsed_seconds)

        resolved_notes = _resolve_melody_overlaps(raw_notes)
        tracks.append(
            MelodyTrack(
                track_id=f"midi:{track_index}",
                name=name,
                emoji=_choose_track_emoji(name, program, main_channel == 9),
                notes=resolved_notes,
                source_notes=list(raw_notes),
                note_total=len(resolved_notes),
                chord_count=_count_chords(resolved_notes),
                duration_ms=duration_ms,
                source_name=midi_path.name,
                program=program,
                channel=main_channel,
                is_json_module=False,
            )
        )

    return tracks, bpm


def build_monitor_values(
    frame: Optional[DecodedFrame],
    stats: Dict[str, int],
    rate_hz: float,
    connected: bool,
    error: str,
) -> Dict[str, str]:
    level, status_text, hint = link_status_summary(connected, error)
    values = {
        "serial.status": status_text,
        "serial.hint": hint,
        "serial.rate": f"{rate_hz:.1f} Hz",
        "serial.crc_rate": f"{crc_error_rate(stats):.2f}%",
        "serial.good_frames": str(stats.get("good", 0)),
        "serial.total_frames": str(stats.get("total", 0)),
    }

    if frame is None:
        placeholders = [
            "frame.timestamp",
            "frame.seq",
            "frame.status_err",
            "frame.payload_hex",
            "status.controller",
            "status.err_flags",
            "mode.output_if",
            "mode.pose_mode",
            "mode.attitude_format",
            "mode.flags",
            "input.key_flags",
            "input.delta_key",
            "input.wheel_pos",
            "input.joy_x",
            "input.joy_y",
            "motion.pos_x",
            "motion.pos_y",
            "motion.pos_z",
            "motion.pos_half",
            "motion.euler_roll",
            "motion.euler_pitch",
            "motion.euler_yaw",
            "motion.quat_w",
            "motion.quat_x",
            "motion.quat_y",
            "motion.quat_z",
            "motion.att_half",
            "encoder.group",
            "encoder.1.raw",
            "encoder.1.deg",
            "encoder.2.raw",
            "encoder.2.deg",
            "encoder.3.raw",
            "encoder.3.deg",
            "payload.byte0",
            "payload.bytes1_2",
            "payload.byte3",
            "payload.bytes4_5",
            "payload.byte6",
            "payload.byte7",
            "payload.bytes8_13",
            "payload.bytes14_21",
            "payload.byte22",
            "payload.bytes23_28",
            "payload.byte29",
        ]
        for key in placeholders:
            values[key] = "--"
        for index in range(1, 9):
            values[f"input.key{index}"] = "--"
        return values

    payload = frame.payload_bytes
    values.update(
        {
            "frame.timestamp": time.strftime("%H:%M:%S", time.localtime(frame.timestamp))
            + f".{int((frame.timestamp % 1.0) * 1000):03d}",
            "frame.seq": str(frame.seq),
            "frame.status_err": f"0x{frame.status_err_byte:02X}",
            "frame.payload_hex": frame.payload_hex,
            "status.controller": controller_status_name(frame.status),
            "status.err_flags": f"{err_flags_text(frame.err_flags)} (0x{frame.err_flags:01X})",
            "mode.output_if": frame.output_if,
            "mode.pose_mode": frame.pose_mode,
            "mode.attitude_format": frame.attitude_format,
            "mode.flags": f"0x{frame.mode_flags:02X}",
            "input.key_flags": f"0x{frame.key_flags:04X}",
            "input.delta_key": str(frame.delta_key),
            "input.wheel_pos": str(frame.wheel_pos),
            "input.joy_x": str(frame.joy_x),
            "input.joy_y": str(frame.joy_y),
            "motion.pos_x": f"{frame.pos[0]:+.3f} mm",
            "motion.pos_y": f"{frame.pos[1]:+.3f} mm",
            "motion.pos_z": f"{frame.pos[2]:+.3f} mm",
            "motion.pos_half": " ".join(f"0x{word:04X}" for word in frame.pos_half_raw),
            "motion.euler_roll": f"{frame.euler[0]:+.3f} deg",
            "motion.euler_pitch": f"{frame.euler[1]:+.3f} deg",
            "motion.euler_yaw": f"{frame.euler[2]:+.3f} deg",
            "motion.quat_w": f"{frame.quat[0]:+.5f}",
            "motion.quat_x": f"{frame.quat[1]:+.5f}",
            "motion.quat_y": f"{frame.quat[2]:+.5f}",
            "motion.quat_z": f"{frame.quat[3]:+.5f}",
            "motion.att_half": " ".join(f"0x{word:04X}" for word in frame.att_half_raw),
            "encoder.group": f"[{frame.enc_raw[0]}, {frame.enc_raw[1]}, {frame.enc_raw[2]}]",
            "encoder.1.raw": str(frame.enc_raw[0]),
            "encoder.1.deg": f"{calibrated_raw_to_deg(frame.enc_raw[0]):+.2f} deg",
            "encoder.2.raw": str(frame.enc_raw[1]),
            "encoder.2.deg": f"{calibrated_raw_to_deg(frame.enc_raw[1]):+.2f} deg",
            "encoder.3.raw": str(frame.enc_raw[2]),
            "encoder.3.deg": f"{calibrated_raw_to_deg(frame.enc_raw[2]):+.2f} deg",
            "payload.byte0": payload_slice_hex(payload, 0, 1),
            "payload.bytes1_2": payload_slice_hex(payload, 1, 3),
            "payload.byte3": payload_slice_hex(payload, 3, 4),
            "payload.bytes4_5": payload_slice_hex(payload, 4, 6),
            "payload.byte6": payload_slice_hex(payload, 6, 7),
            "payload.byte7": payload_slice_hex(payload, 7, 8),
            "payload.bytes8_13": payload_slice_hex(payload, 8, 14),
            "payload.bytes14_21": payload_slice_hex(payload, 14, 22),
            "payload.byte22": payload_slice_hex(payload, 22, 23),
            "payload.bytes23_28": payload_slice_hex(payload, 23, 29),
            "payload.byte29": payload_slice_hex(payload, 29, 30),
        }
    )

    for index in range(1, 9):
        pressed = 1 if frame.key_flags & (1 << (index - 1)) else 0
        values[f"input.key{index}"] = str(pressed)

    return values


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
        decoded: List[DecodedFrame] = []

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
            quat = tuple(half_to_float(word) for word in att_h)
            euler = (0.0, 0.0, 0.0)
        else:
            euler = (half_to_float(att_h[0]), half_to_float(att_h[1]), half_to_float(att_h[2]))
            quat = (1.0, 0.0, 0.0, 0.0)

        return DecodedFrame(
            timestamp=time.time(),
            seq=seq,
            status=(status_err >> 4) & 0x0F,
            err_flags=status_err & 0x0F,
            status_err_byte=status_err,
            key_flags=key_flags,
            delta_key=delta_key,
            wheel_pos=wheel_pos,
            joy_x=joy_x,
            joy_y=joy_y,
            pos=(half_to_float(pos_h[0]), half_to_float(pos_h[1]), half_to_float(pos_h[2])),
            pos_half_raw=(pos_h[0], pos_h[1], pos_h[2]),
            euler=euler,
            quat=(quat[0], quat[1], quat[2], quat[3]),
            att_half_raw=(att_h[0], att_h[1], att_h[2], att_h[3]),
            attitude_format=attitude_format,
            output_if=output_if,
            pose_mode=pose_mode,
            mode_flags=mode_flags,
            enc_raw=(enc_raw[0], enc_raw[1], enc_raw[2]),
            reserved=payload[29],
            payload_bytes=payload,
            payload_hex=payload.hex(" "),
        )


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baud: int) -> None:
        super().__init__(daemon=True)
        self._port = port
        self._baud = baud
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._ser = None

        self._parser = RmFrameParser()
        self._latest: Optional[DecodedFrame] = None
        self._stats = self._parser.stats()
        self._times: Deque[float] = deque(maxlen=300)
        self._connected = False
        self._error = ""
        self._line_buffer = bytearray()
        self._xr_status = XrDeviceStatus()
        self._xr_lines: Deque[str] = deque(maxlen=32)

    def stop_worker(self) -> None:
        self._stop_event.set()
        self.join(timeout=1.0)

    def snapshot(
        self,
    ) -> Tuple[
        Optional[DecodedFrame],
        Dict[str, int],
        float,
        bool,
        str,
        XrDeviceStatus,
        List[str],
    ]:
        with self._lock:
            latest = self._latest
            stats = dict(self._stats)
            times = list(self._times)
            connected = self._connected
            error = self._error
            xr_status = XrDeviceStatus(**vars(self._xr_status))
            xr_lines = list(self._xr_lines)

        rate_hz = 0.0
        if len(times) >= 2:
            dt = times[-1] - times[0]
            if dt > 1e-6:
                rate_hz = (len(times) - 1) / dt

        return latest, stats, rate_hz, connected, error, xr_status, xr_lines

    def send_command(self, line: str) -> bool:
        payload = (line.rstrip("\r\n") + "\n").encode("ascii", errors="ignore")
        with self._lock:
            ser = self._ser
        if ser is None:
            return False
        try:
            ser.write(payload)
            ser.flush()
            return True
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
            return False

    def _consume_side_channel(self, chunk: bytes) -> None:
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
                    status = parse_xr_status_line(line)
                    with self._lock:
                        self._xr_lines.append(line)
                        if status is not None:
                            self._xr_status = status
                continue

            if 32 <= byte <= 126:
                if len(self._line_buffer) < 180:
                    self._line_buffer.append(byte)
                else:
                    self._line_buffer.clear()
            else:
                self._line_buffer.clear()

    def run(self) -> None:
        if serial is None:
            with self._lock:
                self._error = "缺少 pyserial"
            return

        ser = None
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.05)
            with self._lock:
                self._ser = ser
                self._connected = True
                self._error = ""

            while not self._stop_event.is_set():
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue

                self._consume_side_channel(chunk)
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
                self._ser = None
                self._connected = False


@dataclass
class XrBridgeSnapshot:
    running: bool = False
    serial_port: str = ""
    serial_baud: int = 2000000
    serial_connected: bool = False
    bridge_host: str = "127.0.0.1"
    bridge_port: int = 8791
    client_connected: bool = False
    frames_received: int = 0
    frames_forwarded: int = 0
    receive_rate_hz: float = 0.0
    forward_rate_hz: float = 0.0
    last_seq: int = 0
    last_packet_age_ms: int = 0
    pending_seq: int = 0
    last_error: str = ""
    xr_status: XrDeviceStatus = field(default_factory=XrDeviceStatus)
    xr_lines: List[str] = field(default_factory=list)


class XrUartBridgeManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._serial = None
        self._serial_port = ""
        self._serial_baud = 2000000
        self._bridge_host = "127.0.0.1"
        self._bridge_port = 8791
        self._running = False
        self._serial_connected = False
        self._client_connected = False
        self._last_error = ""
        self._xr_status = XrDeviceStatus()
        self._xr_lines: Deque[str] = deque(maxlen=32)
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
            return False, "请先选择 ESP32 USB 串口"

        restart_needed = False
        with self._lock:
            restart_needed = (
                (self._thread is not None and self._thread.is_alive())
                and (
                    self._serial_port != serial_port
                    or self._serial_baud != serial_baud
                    or self._bridge_host != bridge_host
                    or self._bridge_port != bridge_port
                )
            )

        if restart_needed:
            self.stop()

        with self._lock:
            self._serial_port = serial_port
            self._serial_baud = serial_baud
            self._bridge_host = bridge_host
            self._bridge_port = bridge_port
            already_running = self._thread is not None and self._thread.is_alive()

        if already_running:
            return True, f"XR 串口桥已运行：{serial_port}"

        self._stop_event = threading.Event()
        thread_rx = threading.Thread(target=self._run_loop, daemon=True)
        thread_tx = threading.Thread(target=self._tx_loop, daemon=True)
        with self._lock:
            self._thread = thread_rx
            self._thread_tx = thread_tx
            self._running = True
            self._last_error = ""
            self._packet_buffer.clear()
            self._packet_queue.clear()
            self._line_buffer.clear()
        thread_rx.start()
        thread_tx.start()
        return True, f"XR 串口桥已启动：{serial_port}"

    def stop(self) -> None:
        self._stop_event.set()
        thread_rx = None
        thread_tx = None
        with self._lock:
            thread_rx = self._thread
            thread_tx = getattr(self, "_thread_tx", None)
        if thread_rx is not None:
            thread_rx.join(timeout=1.2)
        if thread_tx is not None:
            thread_tx.join(timeout=1.2)
        with self._lock:
            self._thread = None
            self._thread_tx = None
            self._running = False

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

    def snapshot(self) -> XrBridgeSnapshot:
        with self._lock:
            times_rx = list(self._rx_times)
            times_tx = list(self._tx_times)
            age_ms = int(max(0.0, (time.time() - self._last_packet_at) * 1000.0)) if self._last_packet_at > 0 else 0
            return XrBridgeSnapshot(
                running=self._running,
                serial_port=self._serial_port,
                serial_baud=self._serial_baud,
                serial_connected=self._serial_connected,
                bridge_host=self._bridge_host,
                bridge_port=self._bridge_port,
                client_connected=self._client_connected,
                frames_received=self._frames_received,
                frames_forwarded=self._frames_forwarded,
                receive_rate_hz=self._rate_hz(times_rx),
                forward_rate_hz=self._rate_hz(times_tx),
                last_seq=self._last_seq,
                last_packet_age_ms=age_ms,
                pending_seq=self._pending_seq,
                last_error=self._last_error,
                xr_status=XrDeviceStatus(**vars(self._xr_status)),
                xr_lines=list(self._xr_lines),
            )

    @staticmethod
    def _rate_hz(times: List[float]) -> float:
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        if dt <= 1e-6:
            return 0.0
        return (len(times) - 1) / dt

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
                    status = parse_xr_status_line(line)
                    with self._lock:
                        self._xr_lines.append(line)
                        if status is not None:
                            self._xr_status = status
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
            ser = serial.Serial(port, baud, timeout=0.01)
            with self._lock:
                self._serial = ser
                self._serial_connected = True
                self._last_error = ""
        except Exception as exc:
            with self._lock:
                self._serial_connected = False
                self._last_error = str(exc)
            time.sleep(0.2)

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
            ser_ready = self._serial is not None
        if packet and seq and ser_ready:
            self._forward_packet(packet, seq)

    def _consume_bridge_bytes(self, chunk: bytes) -> None:
        if not chunk:
            return
        self._packet_buffer.extend(chunk)
        while len(self._packet_buffer) >= XR_UART_PACKET_SIZE:
            packet = bytes(self._packet_buffer[:XR_UART_PACKET_SIZE])
            del self._packet_buffer[:XR_UART_PACKET_SIZE]
            magic, version, _flags, seq = struct.unpack_from("<IHHI", packet, 0)
            if magic != 0x31525843 or version != 1:
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
            """Runs at 60Hz, dequeues ONLY the newest packet to ESP32."""
            target_interval = 1.0 / 60.0
            
            while not self._stop_event.is_set():
                start_time = time.perf_counter()
                
                packet = None
                seq = 0
                with self._lock:
                    if self._packet_queue:
                        # 【核心修复 1】：LIFO (后进先出) 永远只取最新的一帧
                        packet, seq = self._packet_queue[-1]
                        # 清空积压的历史包，彻底消灭 Bufferbloat 造成的 100+ms 延迟
                        self._packet_queue.clear()
                        
                if packet is not None:
                    self._forward_packet(packet, seq)
                    
                # 【核心修复 2】：高精度自旋睡眠，替代不准的 time.sleep()
                # 计算处理开销，得出还需等待的时间
                elapsed = time.perf_counter() - start_time
                sleep_time = target_interval - elapsed
                
                if sleep_time > 0:
                    end_time = time.perf_counter() + sleep_time
                    # 使用 perf_counter 配合极短时的 yield，实现精确的 60Hz 节奏
                    while time.perf_counter() < end_time:
                        time.sleep(0) # 让出 CPU 时间片，防止单核占满

    def _run_loop(self) -> None:
        server = None
        client = None
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            with self._lock:
                bind_host = self._bridge_host
                bind_port = self._bridge_port
            server.bind((bind_host, bind_port))
            server.listen(1)
            server.settimeout(0.05)

            while not self._stop_event.is_set():
                self._ensure_serial()
                self._flush_pending_packet()

                if client is None:
                    try:
                        candidate, _addr = server.accept()
                        candidate.settimeout(0.02)
                        candidate.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        client = candidate
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
                        chunk = client.recv(4096)
                        if chunk:
                            self._consume_bridge_bytes(chunk)
                        else:
                            client.close()
                            client = None
                            with self._lock:
                                self._client_connected = False
                    except socket.timeout:
                        pass
                    except Exception as exc:
                        with self._lock:
                            self._last_error = str(exc)
                            self._client_connected = False
                        try:
                            client.close()
                        except Exception:
                            pass
                        client = None

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
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            if server is not None:
                try:
                    server.close()
                except Exception:
                    pass
            self._close_serial()
            with self._lock:
                self._client_connected = False
                self._running = False


if QT_IMPORT_ERROR is None and QFLUENT_IMPORT_ERROR is None:

    def effective_theme() -> Theme:
        return Theme.DARK if isDarkTheme() else Theme.LIGHT


    def themed_icon_pixmap(icon, size: int, color: Optional[QColor] = None):
        if hasattr(icon, "icon"):
            qicon = icon.icon(theme=effective_theme(), color=color)
        else:
            qicon = QIcon(icon)
        return qicon.pixmap(size, size)


    def emoji_icon(emoji: str, size: int = 28) -> QIcon:
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.TextAntialiasing)
        font = QFont()
        font.setPointSize(max(12, int(size * 0.55)))
        painter.setFont(font)
        painter.drawText(pixmap.rect(), Qt.AlignCenter, emoji)
        painter.end()
        return QIcon(pixmap)


    def build_scrollable_page(page: QWidget) -> QWidget:
        outer = QVBoxLayout(page)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea(page)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setAutoFillBackground(False)
        scroll.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        scroll.viewport().setAutoFillBackground(False)
        scroll.viewport().setStyleSheet("background: transparent;")
        outer.addWidget(scroll)

        content = QWidget(scroll)
        content.setAutoFillBackground(False)
        content.setAttribute(Qt.WA_StyledBackground, False)
        content.setStyleSheet("background: transparent;")
        scroll.setWidget(content)
        page._scrollArea = scroll  # type: ignore[attr-defined]
        page._scrollContent = content  # type: ignore[attr-defined]
        return content


    class ConnectionStatusNavItem(NavigationWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(isSelectable=False, parent=parent)
            self._text = "未连接"
            self._connected = False
            setFont(self)
            self.setToolTip(self._text)

        def setStatus(self, connected: bool, output_if: Optional[str]) -> None:
            self._connected = connected
            if connected:
                iface = "RS232" if output_if == "RS232" else "USB"
                self._text = f"已连接：{iface}"
            else:
                self._text = "未连接"

            self.setToolTip(self._text)
            self.update()

        def paintEvent(self, e) -> None:  # type: ignore[override]
            _ = e
            painter = QPainter(self)
            painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)
            painter.setPen(Qt.NoPen)

            if self.isPressed:
                painter.setOpacity(0.7)
            if not self.isEnabled():
                painter.setOpacity(0.4)

            c = 255 if isDarkTheme() else 0
            margins = self._margins()
            left_margin, right_margin = margins.left(), margins.right()
            if self.isEnter and self.isEnabled():
                painter.setBrush(QColor(c, c, c, 10))
                painter.drawRoundedRect(self.rect(), 5, 5)

            dot_color = QColor("#2F9E44" if self._connected else "#D13438")
            if self.isCompacted:
                dot_rect = QRectF((self.width() - 10) / 2, (self.height() - 10) / 2, 10, 10)
                painter.setBrush(dot_color)
                painter.drawEllipse(dot_rect)
                return

            dot_rect = QRectF(14.5 + left_margin, (self.height() - 10) / 2, 10, 10)
            painter.setBrush(dot_color)
            painter.drawEllipse(dot_rect)

            painter.setPen(self.textColor())
            painter.setFont(self.font())
            text_left = 44 + left_margin
            painter.drawText(
                QRectF(text_left, 0, self.width() - 13 - text_left - right_margin, self.height()),
                Qt.AlignVCenter,
                self._text,
            )


    class SectionHeader(QWidget):
        def __init__(self, icon, title: str, description: str = "", parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.icon = icon
            layout = QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(2)

            top = QHBoxLayout()
            top.setContentsMargins(0, 0, 0, 0)
            top.setSpacing(8)

            titleLabel = StrongBodyLabel(title, self)
            self.iconLabel: Optional[QLabel] = None
            if icon is not None:
                self.iconLabel = QLabel(self)
                self._applyIcon()
                top.addWidget(self.iconLabel)
            top.addWidget(titleLabel)
            top.addStretch(1)

            layout.addLayout(top)
            if description:
                descLabel = CaptionLabel(description, self)
                descLabel.setWordWrap(True)
                layout.addWidget(descLabel)

            qconfig.themeChangedFinished.connect(self._applyIcon)

        def _applyIcon(self, *_args) -> None:
            if self.iconLabel is not None:
                self.iconLabel.setPixmap(themed_icon_pixmap(self.icon, 18))


    class StatusBadge(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.iconLabel = QLabel(self)
            self.textLabel = BodyLabel("--", self)
            self.level = "neutral"
            self.statusText = "未连接"
            self.textLabel.setStyleSheet("font-weight: 600;")

            layout = QHBoxLayout(self)
            layout.setContentsMargins(10, 4, 10, 4)
            layout.setSpacing(6)
            layout.addWidget(self.iconLabel)
            layout.addWidget(self.textLabel)
            self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            self.setStatus("neutral", "未连接")
            qconfig.themeChangedFinished.connect(self._applyStyle)

        def setStatus(self, level: str, text: str) -> None:
            self.level = level
            self.statusText = text
            self._applyStyle()

        def _applyStyle(self, *_args) -> None:
            if isDarkTheme():
                style_map = {
                    "success": (FIF.COMPLETED, "#6CCB5F", "#17361B", "#2F7D38"),
                    "error": (FIF.CLOSE, "#FF8D85", "#401916", "#C55349"),
                    "neutral": (FIF.INFO, "#D8D8D8", "transparent", "#5C5C5C"),
                }
            else:
                style_map = {
                    "success": (FIF.COMPLETED, "#0E7A0D", "#E7F6EA", "#0E7A0D"),
                    "error": (FIF.CLOSE, "#C42B1C", "#FDECEA", "#C42B1C"),
                    "neutral": (FIF.INFO, "#5A5A5A", "transparent", "#D0D0D0"),
                }

            icon, fg, bg, border = style_map.get(self.level, style_map["neutral"])
            self.iconLabel.setPixmap(themed_icon_pixmap(icon, 14, QColor(fg)))
            self.textLabel.setText(self.statusText)
            self.textLabel.setStyleSheet(f"font-weight: 600; color: {fg};")
            self.setStyleSheet(
                f"StatusBadge {{border: 1px solid {border}; border-radius: 13px; background: {bg};}}"
            )


    class SerialStatusBar(QFrame):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self._level = "neutral"
            self._title = "未连接"
            self._content = "请选择串口并开始监视"
            self._icon = InfoBarIcon.INFORMATION
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            self.setMinimumWidth(0)
            self.setObjectName("SerialStatusBar")

            self.iconWidget = InfoIconWidget(self._icon, self)
            self.titleLabel = QLabel(self._title, self)
            self.titleLabel.setWordWrap(True)
            self.contentLabel = QLabel(self._content, self)
            self.contentLabel.setWordWrap(True)
            self.titleLabel.setObjectName("titleLabel")
            self.contentLabel.setObjectName("contentLabel")
            probe = InfoBar(
                InfoBarIcon.INFORMATION,
                "未连接",
                "请选择串口并开始监视",
                orient=Qt.Vertical,
                isClosable=False,
                duration=-1,
                position=InfoBarPosition.NONE,
            )
            self.titleLabel.setFont(probe.titleLabel.font())
            self.contentLabel.setFont(probe.contentLabel.font())
            probe.deleteLater()

            self.hBoxLayout = QHBoxLayout(self)
            self.hBoxLayout.setContentsMargins(12, 12, 12, 12)
            self.hBoxLayout.setSpacing(0)
            self.hBoxLayout.addWidget(self.iconWidget, 0, Qt.AlignTop | Qt.AlignLeft)

            self.textLayout = QVBoxLayout()
            self.textLayout.setContentsMargins(1, 7, 0, 4)
            self.textLayout.setSpacing(2)
            self.textLayout.addWidget(self.titleLabel)
            self.textLayout.addWidget(self.contentLabel)
            self.hBoxLayout.addLayout(self.textLayout, 1)

            self.setStatus(self._level, self._title, self._content)
            qconfig.themeChangedFinished.connect(self._applyAppearance)

        def _applyAppearance(self, *_args) -> None:
            if isDarkTheme():
                style_map = {
                    "success": (InfoBarIcon.SUCCESS, QColor("#6CCB5F"), "#17361B", "#F5F9F5", "#D1E7D3"),
                    "error": (InfoBarIcon.ERROR, QColor("#FF8D85"), "#401916", "#FFF4F3", "#F6C8C2"),
                    "neutral": (InfoBarIcon.INFORMATION, QColor("#7AB8FF"), "#1B314A", "#F3F8FF", "#C7DDF9"),
                }
            else:
                style_map = {
                    "success": (InfoBarIcon.SUCCESS, QColor("#0E7A0D"), "#E8F7EC", "#152514", "#466044"),
                    "error": (InfoBarIcon.ERROR, QColor("#C42B1C"), "#FDEDEA", "#2D1615", "#6B4945"),
                    "neutral": (InfoBarIcon.INFORMATION, QColor("#0F6CBD"), "#EAF3FF", "#142333", "#4D6278"),
                }

            icon, accent, bg, title_fg, content_fg = style_map.get(self._level, style_map["neutral"])
            self._icon = icon
            self.iconWidget.icon = icon
            self.iconWidget.update()
            self.titleLabel.setText(self._title)
            self.contentLabel.setText(self._content)
            self.titleLabel.setStyleSheet(f"color: {title_fg};")
            self.contentLabel.setStyleSheet(f"color: {content_fg};")
            self.setStyleSheet(
                f"QFrame#SerialStatusBar {{ background: {bg}; border: none; border-radius: 6px; }}"
            )

        def setStatus(self, level: str, title: str, content: str) -> None:
            self._level = level
            self._title = title
            self._content = content
            self._applyAppearance()


    class CompactMetricCard(CardWidget):
        def __init__(
            self,
            icon,
            title: str,
            description: str,
            parent: Optional[QWidget] = None,
        ) -> None:
            super().__init__(parent)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            layout = QVBoxLayout(self)
            layout.setContentsMargins(14, 12, 14, 12)
            layout.setSpacing(8)

            layout.addWidget(SectionHeader(icon, title, description, self))

            self.valueLabel = SubtitleLabel("--", self)
            self.valueLabel.setWordWrap(True)
            self.noteLabel = CaptionLabel("", self)
            self.noteLabel.setWordWrap(True)

            layout.addWidget(self.valueLabel)
            layout.addWidget(self.noteLabel)

        def setContent(self, value: str, note: str) -> None:
            self.valueLabel.setText(value)
            self.noteLabel.setText(note)


    class GaugeRow(QWidget):
        def __init__(self, label: str, minimum: int, maximum: int, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.minimum = minimum
            self.maximum = maximum
            layout = QHBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(10)

            self.nameLabel = BodyLabel(label, self)
            self.nameLabel.setFixedWidth(70)

            self.progress = ProgressBar(self)
            self.progress.setRange(minimum, maximum)
            self.progress.setValue(minimum)
            self.progress.setTextVisible(False)
            self.progress.setUseAni(False)
            self.progress.setFixedHeight(8)

            self.valueLabel = CaptionLabel("--", self)
            self.valueLabel.setFixedWidth(78)
            self.valueLabel.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            layout.addWidget(self.nameLabel)
            layout.addWidget(self.progress, 1)
            layout.addWidget(self.valueLabel)

        def setValue(self, value: int, text: str) -> None:
            clamped = max(self.minimum, min(self.maximum, int(value)))
            self.progress.setValue(clamped)
            self.valueLabel.setText(text)


    class IndicatorDot(QWidget):
        def __init__(self, label: str, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.active = False
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

            layout = QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(6)
            layout.setAlignment(Qt.AlignCenter)

            self.dot = QLabel(self)
            self.dot.setFixedSize(16, 16)
            self.text = CaptionLabel(label, self)
            self.text.setAlignment(Qt.AlignCenter)

            layout.addWidget(self.dot, 0, Qt.AlignCenter)
            layout.addWidget(self.text)
            self.setActive(False)
            qconfig.themeChangedFinished.connect(self._applyStyle)

        def setActive(self, active: bool) -> None:
            self.active = active
            self._applyStyle()

        def _applyStyle(self, *_args) -> None:
            if self.active:
                fill = "#2F9E44" if isDarkTheme() else "#0E7A0D"
                border = fill
            else:
                fill = "#4E4E4E" if isDarkTheme() else "#C8C8C8"
                border = "#747474" if isDarkTheme() else "#9D9D9D"
            self.dot.setStyleSheet(
                f"border-radius: 8px; background: {fill}; border: 1px solid {border};"
            )


    class KeyDotsStrip(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            layout = QHBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(8)

            self.dots: List[IndicatorDot] = []
            for index in range(KEY_DOT_COUNT):
                dot = IndicatorDot(f"K{index + 1}", self)
                self.dots.append(dot)
                layout.addWidget(dot, 1)

        def setFlags(self, key_flags: int) -> None:
            for index, dot in enumerate(self.dots):
                dot.setActive(bool(key_flags & (1 << index)))


    class CompactGroupCard(CardWidget):
        def __init__(self, icon, title: str, description: str, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.bodyLayout = QVBoxLayout(self)
            self.bodyLayout.setContentsMargins(14, 12, 14, 12)
            self.bodyLayout.setSpacing(12)
            self.bodyLayout.addWidget(SectionHeader(icon, title, description, self))

        def addWidget(self, widget: QWidget, stretch: int = 0) -> None:
            self.bodyLayout.addWidget(widget, stretch)

        def addLayout(self, layout, stretch: int = 0) -> None:
            self.bodyLayout.addLayout(layout, stretch)


    class EncoderVisualCard(CompactGroupCard):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(FIF.ROTATE, "编码器值", "", parent)
            self.rows = [GaugeRow(f"编码器{i}", ENCODER_MIN, ENCODER_MAX, self) for i in range(1, 4)]
            for row in self.rows:
                self.addWidget(row)

        def updateValues(self, frame: Optional[DecodedFrame]) -> None:
            values = frame.enc_raw if frame is not None else (0, 0, 0)
            for index, row in enumerate(self.rows):
                raw = values[index]
                row.setValue(raw, str(raw) if frame is not None else "--")


    class JoystickVisualCard(CompactGroupCard):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(FIF.IOT, "摇杆值", "", parent)
            self.rows = [
                GaugeRow("摇杆 X", JOYSTICK_MIN, JOYSTICK_MAX, self),
                GaugeRow("摇杆 Y", JOYSTICK_MIN, JOYSTICK_MAX, self),
            ]
            for row in self.rows:
                self.addWidget(row)

        def updateValues(self, frame: Optional[DecodedFrame]) -> None:
            if frame is None:
                self.rows[0].setValue(0, "--")
                self.rows[1].setValue(0, "--")
                return
            self.rows[0].setValue(frame.joy_x, str(frame.joy_x))
            self.rows[1].setValue(frame.joy_y, str(frame.joy_y))


    class KeyVisualCard(CompactGroupCard):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(FIF.CHECKBOX, "按键状态", "", parent)
            header = self.bodyLayout.itemAt(0).widget()
            if header is not None:
                header.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
                header.setFixedHeight(header.sizeHint().height())

            self.bodyLayout.setSpacing(0)
            self.strip = KeyDotsStrip(self)
            self.bodyLayout.addSpacing(6)
            self.bodyLayout.addStretch(1)
            self.bodyLayout.addWidget(self.strip)
            self.bodyLayout.addStretch(1)

        def updateValues(self, frame: Optional[DecodedFrame]) -> None:
            self.strip.setFlags(frame.key_flags if frame is not None else 0)


    class WheelVisualCard(CompactGroupCard):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(FIF.SYNC, "滚轮累计值", "", parent)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

            self.valueLabel = TitleLabel("--", self)
            self.valueLabel.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.noteLabel = CaptionLabel("等待串口数据", self)
            self.noteLabel.setWordWrap(True)

            self.addWidget(self.valueLabel)
            self.addWidget(self.noteLabel)

        def updateValue(self, frame: Optional[DecodedFrame]) -> None:
            if frame is None:
                self.valueLabel.setText("--")
                self.noteLabel.setText("等待串口数据")
                return

            self.valueLabel.setText(str(frame.wheel_pos))
            self.noteLabel.setText("payload[4:5] wheelPos")


    class OverviewPage(QWidget):
        def __init__(self, port: str, baud: int, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.setObjectName("overview-page")

            content = build_scrollable_page(self)
            root = QHBoxLayout(content)
            root.setContentsMargins(24, 20, 24, 24)
            root.setSpacing(16)

            left = QVBoxLayout()
            left.setSpacing(12)
            right = QVBoxLayout()
            right.setSpacing(12)

            root.addLayout(left, 5)
            root.addLayout(right, 7)

            self.connectCard = CompactGroupCard(FIF.CONNECT, "串口连接", "", self)
            form = QGridLayout()
            form.setContentsMargins(0, 0, 0, 0)
            form.setHorizontalSpacing(10)
            form.setVerticalSpacing(10)

            self.portCombo = ComboBox(self.connectCard)
            self.portCombo.setMinimumWidth(190)
            self.refreshPortsButton = TransparentToolButton(FIF.SYNC, self.connectCard)
            self.refreshPortsButton.setToolTip("刷新串口列表")

            self.baudCombo = ComboBox(self.connectCard)
            self.baudCombo.addItems(list(BAUD_OPTIONS))
            self.baudCombo.setCurrentText(str(baud))

            self.connectButton = PrimaryPushButton("连接", self.connectCard)
            self.connectButton.setIcon(FIF.CONNECT)
            self.disconnectButton = PushButton("断开", self.connectCard)
            self.disconnectButton.setIcon(FIF.CANCEL)

            form.addWidget(CaptionLabel("串口", self.connectCard), 0, 0)
            form.addWidget(self.portCombo, 0, 1)
            form.addWidget(self.refreshPortsButton, 0, 2)
            form.addWidget(CaptionLabel("波特率", self.connectCard), 1, 0)
            form.addWidget(self.baudCombo, 1, 1, 1, 2)
            form.addWidget(self.connectButton, 2, 1)
            form.addWidget(self.disconnectButton, 2, 2)
            form.setColumnStretch(1, 1)
            self.connectCard.addLayout(form)
            left.addWidget(self.connectCard)

            self.serialStateBar = SerialStatusBar(self)
            left.addWidget(self.serialStateBar)

            self.linkMetricsCard = CompactGroupCard(FIF.SPEED_HIGH, "串口指标", "", self)
            self.rateValueLabel = SubtitleLabel("0.0 Hz", self.linkMetricsCard)
            self.rateValueLabel.setStyleSheet("font-weight: 600;")
            self.crcLabel = CaptionLabel("CRC错误率 0.00%", self.linkMetricsCard)
            self.linkMetricsCard.addWidget(self.rateValueLabel)
            self.linkMetricsCard.addWidget(self.crcLabel)
            left.addWidget(self.linkMetricsCard)
            left.addStretch(1)

            metricsGrid = QGridLayout()
            metricsGrid.setContentsMargins(0, 0, 0, 0)
            metricsGrid.setHorizontalSpacing(12)
            metricsGrid.setVerticalSpacing(12)
            right.addLayout(metricsGrid)

            self.outputModeCard = CompactMetricCard(FIF.SEND, "输出模式", "", self)
            self.errorCard = CompactMetricCard(FIF.INFO, "错误标志", "", self)
            self.controllerCard = CompactMetricCard(FIF.ROBOT, "控制器状态", "", self)
            self.attitudeCard = CompactMetricCard(FIF.ALIGNMENT, "姿态模式", "", self)

            metricsGrid.addWidget(self.outputModeCard, 0, 0)
            metricsGrid.addWidget(self.errorCard, 0, 1)
            metricsGrid.addWidget(self.controllerCard, 1, 0)
            metricsGrid.addWidget(self.attitudeCard, 1, 1)

            self.encoderVisualCard = EncoderVisualCard(self)
            self.keyVisualCard = KeyVisualCard(self)
            self.wheelVisualCard = WheelVisualCard(self)
            self.joystickVisualCard = JoystickVisualCard(self)
            right.addWidget(self.encoderVisualCard)

            keyWheelRow = QHBoxLayout()
            keyWheelRow.setContentsMargins(0, 0, 0, 0)
            keyWheelRow.setSpacing(12)
            keyWheelRow.addWidget(self.keyVisualCard, 5)
            keyWheelRow.addWidget(self.wheelVisualCard, 3)
            right.addLayout(keyWheelRow)

            right.addWidget(self.joystickVisualCard)
            right.addStretch(1)

            self.refreshPorts()
            if port:
                self._trySelectPort(port)

        def refreshPorts(self) -> None:
            current = self.selectedPort()
            self.portCombo.clear()

            ports: List[str] = []
            if list_ports is not None:
                ports = [info.device for info in list_ports.comports()]

            if not ports:
                self.portCombo.addItem("未检测到可用串口")
                if hasattr(self.portCombo, "setItemEnabled"):
                    self.portCombo.setItemEnabled(0, False)
                return

            self.portCombo.addItems(ports)
            if current:
                self._trySelectPort(current)

        def _trySelectPort(self, port: str) -> None:
            index = self.portCombo.findText(port)
            if index >= 0:
                self.portCombo.setCurrentIndex(index)

        def selectedPort(self) -> str:
            text = self.portCombo.currentText().strip()
            if text.startswith("未检测到"):
                return ""
            return text

        def selectedBaud(self) -> int:
            return int(self.baudCombo.currentText())

        def updateView(
            self,
            frame: Optional[DecodedFrame],
            stats: Dict[str, int],
            rate_hz: float,
            connected: bool,
            error: str,
        ) -> None:
            level, text, hint = link_status_summary(connected, error)
            self.serialStateBar.setStatus(level, text, hint)

            self.rateValueLabel.setText(f"{rate_hz:.1f} Hz")
            self.crcLabel.setText(f"CRC错误率 {crc_error_rate(stats):.2f}%")

            if frame is None:
                self.outputModeCard.setContent("--", "等待主控输出数据")
                self.errorCard.setContent(err_flags_text(0), "未收到错误标志")
                self.controllerCard.setContent("空闲", "尚未收到有效帧")
                self.attitudeCard.setContent("--", "等待位姿模式")
            else:
                self.outputModeCard.setContent(frame.output_if, f"modeFlags = 0x{frame.mode_flags:02X}")
                self.errorCard.setContent(err_flags_text(frame.err_flags), f"statusErr = 0x{frame.status_err_byte:02X}")
                self.controllerCard.setContent(controller_status_name(frame.status), f"seq = {frame.seq}")
                self.attitudeCard.setContent(
                    f"{frame.pose_mode} / {frame.attitude_format}",
                    "位姿参考 + 姿态格式",
                )

            self.encoderVisualCard.updateValues(frame)
            self.keyVisualCard.updateValues(frame)
            self.wheelVisualCard.updateValue(frame)
            self.joystickVisualCard.updateValues(frame)


    class XrControlPage(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.setObjectName("xr-control-page")

            content = build_scrollable_page(self)
            root = QGridLayout(content)
            root.setContentsMargins(16, 12, 16, 16)
            root.setHorizontalSpacing(12)
            root.setVerticalSpacing(10)
            root.setColumnStretch(0, 11)
            root.setColumnStretch(1, 10)
            root.setRowStretch(3, 1)

            self.bridgeCard = CompactGroupCard(FIF.CONNECT, "XR-UART 串口桥", "", self)
            bridgeGrid = QGridLayout()
            bridgeGrid.setContentsMargins(0, 0, 0, 0)
            bridgeGrid.setHorizontalSpacing(10)
            bridgeGrid.setVerticalSpacing(8)

            self.bridgePortCombo = ComboBox(self.bridgeCard)
            self.bridgePortCombo.setMinimumWidth(190)
            self.refreshBridgePortsButton = TransparentToolButton(FIF.SYNC, self.bridgeCard)
            self.refreshBridgePortsButton.setToolTip("刷新 USB 串口列表")
            self.bridgeBaudCombo = ComboBox(self.bridgeCard)
            self.bridgeBaudCombo.addItems(["2000000", "115200"])
            self.bridgeBaudCombo.setCurrentText("2000000")
            self.bridgePortSpin = SpinBox(self.bridgeCard)
            self.bridgePortSpin.setRange(1, 65535)
            self.bridgePortSpin.setFixedWidth(148)
            self.refreshXrStatusButton = PushButton("刷新状态", self.bridgeCard)
            self.enterXrButton = PrimaryPushButton("进入 XR-UART", self.bridgeCard)
            self.exitXrButton = PushButton("退出 XR-UART", self.bridgeCard)

            bridgeGrid.addWidget(CaptionLabel("ESP32 USB 串口", self.bridgeCard), 0, 0)
            bridgeGrid.addWidget(self.bridgePortCombo, 0, 1)
            bridgeGrid.addWidget(self.refreshBridgePortsButton, 0, 2)
            bridgeGrid.addWidget(CaptionLabel("串口波特率", self.bridgeCard), 1, 0)
            bridgeGrid.addWidget(self.bridgeBaudCombo, 1, 1, 1, 2)
            bridgeGrid.addWidget(CaptionLabel("本地桥端口", self.bridgeCard), 2, 0)
            bridgeGrid.addWidget(self.bridgePortSpin, 2, 1)
            bridgeGrid.addWidget(self.refreshXrStatusButton, 3, 1)
            bridgeGrid.addWidget(self.enterXrButton, 3, 2)
            bridgeGrid.addWidget(self.exitXrButton, 4, 2)
            bridgeGrid.setColumnStretch(1, 1)
            self.bridgeCard.addLayout(bridgeGrid)
            root.addWidget(self.bridgeCard, 0, 0)

            self.deviceStatusCard = CompactGroupCard(FIF.ROBOT, "ESP32 XR-UART 状态", "", self)
            self.deviceStatusLabel = BodyLabel("等待 USB 串口桥或状态查询", self.deviceStatusCard)
            self.deviceStatusLabel.setWordWrap(True)
            self.deviceStatusCard.addWidget(self.deviceStatusLabel)
            self.deviceStatusLabel.setMinimumHeight(108)
            root.addWidget(self.deviceStatusCard, 1, 0)

            self.guideCard = CompactGroupCard(FIF.INFO, "Quest 实时数据", "", self)
            self.guideLabel = BodyLabel(
                "等待 Quest 连接。\n"
                "建议流程：1. 选择 ESP32 USB 串口；2. 构建并启动服务；3. Quest 打开右侧地址；4. 进入 XR-UART。",
                self.guideCard,
            )
            self.guideLabel.setWordWrap(True)
            self.guideLabel.setMinimumHeight(120)
            self.guideCard.addWidget(self.guideLabel)
            root.addWidget(self.guideCard, 2, 0)
            self.questStatusLabel = self.guideLabel

            self.relayCard = CompactGroupCard(FIF.SEND, "Quest Web 服务", "", self)
            relayGrid = QGridLayout()
            relayGrid.setContentsMargins(0, 0, 0, 0)
            relayGrid.setHorizontalSpacing(10)
            relayGrid.setVerticalSpacing(8)

            self.positionScaleSpin = SpinBox(self.relayCard)
            self.positionScaleSpin.setRange(1, 400)
            self.positionScaleSpin.setValue(100)
            self.rotationScaleSpin = SpinBox(self.relayCard)
            self.rotationScaleSpin.setRange(1, 400)
            self.rotationScaleSpin.setValue(100)
            self.generateCertButton = PushButton("生成证书", self.relayCard)
            self.startRelayButton = PrimaryPushButton("构建并启动服务", self.relayCard)
            self.stopRelayButton = PushButton("停止服务", self.relayCard)

            relayGrid.addWidget(CaptionLabel("位移倍率 (%)", self.relayCard), 0, 0)
            relayGrid.addWidget(self.positionScaleSpin, 0, 1)
            relayGrid.addWidget(CaptionLabel("转动倍率 (%)", self.relayCard), 1, 0)
            relayGrid.addWidget(self.rotationScaleSpin, 1, 1)
            relayGrid.addWidget(self.generateCertButton, 2, 0)
            relayGrid.addWidget(self.startRelayButton, 2, 1)
            relayGrid.addWidget(self.stopRelayButton, 3, 1)
            relayGrid.setColumnStretch(2, 1)
            self.relayCard.addLayout(relayGrid)
            root.addWidget(self.relayCard, 0, 1)

            self.relayStatusCard = CompactGroupCard(FIF.SYNC, "Relay 诊断", "", self)
            self.relayStatusLabel = BodyLabel("服务未启动", self.relayStatusCard)
            self.relayStatusLabel.setWordWrap(True)
            self.relayStatusLabel.setMinimumHeight(140)
            self.relayStatusCard.addWidget(self.relayStatusLabel)
            root.addWidget(self.relayStatusCard, 1, 1)

            self.accessCard = CompactGroupCard(FIF.DOCUMENT, "访问地址与提示", "", self)
            self.accessLabel = BodyLabel("等待服务启动", self.accessCard)
            self.accessLabel.setWordWrap(True)
            self.accessLabel.setMinimumHeight(106)
            self.accessCard.addWidget(self.accessLabel)
            root.addWidget(self.accessCard, 2, 1)

            self.logCard = CompactGroupCard(FIF.DOCUMENT, "服务日志", "", self)
            self.logList = ListWidget(self.logCard)
            self.logCard.addWidget(self.logList, 1)
            self.logList.setMinimumHeight(200)
            root.addWidget(self.logCard, 3, 0, 1, 2)

            self._webKeyEventLogs: Deque[str] = deque(maxlen=80)
            self._lastWebFrameSeq = -1
            self._lastWebKeyFlags: Optional[int] = None

            self.loadRuntimeConfig()
            self.refreshBridgePorts()

        def loadRuntimeConfig(self) -> None:
            config = load_runtime_config()
            self.bridgePortSpin.setValue(int(config.get("bridgePort", 8791) or 8791))
            self.positionScaleSpin.setValue(int(round(float(config.get("positionScale", 1.0) or 1.0) * 100.0)))
            self.rotationScaleSpin.setValue(int(round(float(config.get("rotationScale", 1.0) or 1.0) * 100.0)))

        def refreshBridgePorts(self) -> None:
            current = self.selectedBridgePort()
            self.bridgePortCombo.clear()
            ports = list_serial_port_names()
            if not ports:
                self.bridgePortCombo.addItem("未检测到可用串口")
                if hasattr(self.bridgePortCombo, "setItemEnabled"):
                    self.bridgePortCombo.setItemEnabled(0, False)
                return
            self.bridgePortCombo.addItems(ports)
            if current:
                index = self.bridgePortCombo.findText(current)
                if index >= 0:
                    self.bridgePortCombo.setCurrentIndex(index)

        def runtimeConfig(self) -> Dict[str, object]:
            config = load_runtime_config()
            config["bridgeHost"] = "127.0.0.1"
            config["bridgePort"] = int(self.bridgePortSpin.value())
            config["positionScale"] = round(float(self.positionScaleSpin.value()) / 100.0, 3)
            config["rotationScale"] = round(float(self.rotationScaleSpin.value()) / 100.0, 3)
            return config

        def selectedBridgePort(self) -> str:
            text = self.bridgePortCombo.currentText().strip()
            if text.startswith("未检测到"):
                return ""
            return text

        def selectedBridgeBaud(self) -> int:
            try:
                return int(self.bridgeBaudCombo.currentText())
            except Exception:
                return 2000000

        def applyDeviceStatus(self, status: XrDeviceStatus) -> None:
            self.deviceStatusLabel.setText(
                "\n".join(
                    [
                        f"模式: {status.mode}    请求: {'ON' if status.requested else 'OFF'}",
                        f"主机链路: {'ACTIVE' if status.link_active else 'WAIT'}    Pose: {'READY' if status.has_pose else 'HOLD'}",
                        f"最近帧: seq={status.seq}    poseAge={status.age_ms} ms",
                        f"恢复链路: {'PENDING' if status.restore_pending else 'NO'}",
                    ]
                )
            )

        def applyRelayHealth(self, health: RelayHealth, bridge: XrBridgeSnapshot, logs: List[str]) -> None:
            self._captureWebKeyEvent(health)
            latest_seq = int(health.latest_frame.get("frameSeq", 0) or 0) if isinstance(health.latest_frame, dict) else 0
            seq_lag = max(0, latest_seq - bridge.last_seq)
            self.relayStatusLabel.setText(
                "\n".join(
                    [
                        f"运行: {'ON' if health.running else 'OFF'}    传输: {health.transport}",
                        f"Quest 客户端: {health.clients}    活动会话: {health.active_sessions}",
                        f"本地桥: {health.bridge_host}:{health.bridge_port}    Node->Python: {'OK' if health.bridge_connected else 'WAIT'}",
                        f"USB 串口: {bridge.serial_port or '--'}    Python->ESP32: {'OK' if bridge.serial_connected else 'WAIT'}",
                        f"倍率: 平移 x{health.position_scale:.2f} / 转动 x{health.rotation_scale:.2f}",
                        f"Quest->PC: {health.receive_rate_hz:.1f} Hz    丢序: {health.frames_dropped_seq}",
                        f"PC->ESP32: {health.relay_rate_hz:.1f} Hz    转发: {health.frames_relayed}/{health.frames_received}    写错: {health.bridge_write_errors}",
                        f"ESP 帧: seq={bridge.last_seq}    seqLag={seq_lag}    age={bridge.last_packet_age_ms} ms",
                        f"待补发: seq={bridge.pending_seq}    Python 收包: {bridge.receive_rate_hz:.1f} Hz    下发: {bridge.forward_rate_hz:.1f} Hz",
                        f"错误: {bridge.last_error or health.last_error or '--'}",
                    ]
                )
            )
            access_lines = ["Quest 访问地址："]
            access_lines.extend(health.access_urls if health.access_urls else ["无可用地址"])
            access_lines.extend(
                [
                    "",
                    "提示：USB 串口负责上行 XR 数据，RS232 负责标准 30B 载荷输出。",
                    "若 Quest 有帧但 ESP32 seq 不动，优先检查 USB 串口占用、线缆和设备管理器中的 CDC 端口。",
                ]
            )
            self.accessLabel.setText("\n".join(access_lines))

            self.logList.clear()
            display_logs: List[str] = []
            if self._webKeyEventLogs:
                display_logs.extend(list(self._webKeyEventLogs)[-20:])
            if logs:
                if display_logs:
                    display_logs.append("---- 服务输出 ----")
                display_logs.extend(logs[-20:])
            for line in display_logs[-40:]:
                self.logList.addItem(line)

        def applyQuestStatus(self, health: RelayHealth) -> None:
            latest = health.latest_frame if isinstance(health.latest_frame, dict) else {}
            rel_pos = latest.get("relPositionMm", [0, 0, 0])
            if not isinstance(rel_pos, list) or len(rel_pos) < 3:
                rel_pos = [0, 0, 0]
            rel_quat = latest.get("relQuaternionWxyz", [1, 0, 0, 0])
            if not isinstance(rel_quat, list) or len(rel_quat) < 4:
                rel_quat = [1, 0, 0, 0]

            quest_to_pc = int(latest.get("questToPcMsEstimate", 0) or 0)
            key_flags = int(latest.get("keyFlags", 0) or 0)
            self.questStatusLabel.setText(
                "\n".join(
                    [
                        f"最近 Quest 帧: seq={int(latest.get('frameSeq', 0) or 0)}    age={int(latest.get('ageMs', 0) or 0)} ms",
                        f"右手连接: {'YES' if bool(latest.get('rightConnected', False)) else 'NO'}    摇杆: ({int(latest.get('joyX', 50) or 50)}, {int(latest.get('joyY', 50) or 50)})",
                        f"latestFrame.keyFlags: 0x{key_flags:02X}    {web_key_flags_text(key_flags)}",
                        f"REL 位置(mm): ({float(rel_pos[0]):.1f}, {float(rel_pos[1]):.1f}, {float(rel_pos[2]):.1f})",
                        f"REL 四元数: ({float(rel_quat[0]):+.3f}, {float(rel_quat[1]):+.3f}, {float(rel_quat[2]):+.3f}, {float(rel_quat[3]):+.3f})",
                        f"临时估计: Quest->PC ≈ {quest_to_pc} ms    relayAge={int(latest.get('relayAgeMs', 0) or 0)} ms",
                        "说明：严格的一次延迟与分段丢包统计仍建议后续改成双向时间戳/回执协议。",
                    ]
                )
            )

        def _captureWebKeyEvent(self, health: RelayHealth) -> None:
            latest = health.latest_frame if isinstance(health.latest_frame, dict) else {}
            if not latest:
                return

            try:
                frame_seq = int(latest.get("frameSeq", 0) or 0)
                key_flags = int(latest.get("keyFlags", 0) or 0) & 0xFFFF
                age_ms = int(latest.get("ageMs", 0) or 0)
            except Exception:
                return

            if frame_seq == self._lastWebFrameSeq and key_flags == self._lastWebKeyFlags:
                return

            if self._lastWebKeyFlags is None:
                self._lastWebFrameSeq = frame_seq
                self._lastWebKeyFlags = key_flags
                if key_flags == 0:
                    return
                old_flags = 0
            else:
                old_flags = self._lastWebKeyFlags

            self._lastWebFrameSeq = frame_seq
            if key_flags == old_flags:
                return

            timestamp = time.strftime("%H:%M:%S", time.localtime())
            millis = int((time.time() % 1.0) * 1000.0)
            line = (
                f"[XR-WEB {timestamp}.{millis:03d}] "
                f"seq={frame_seq} keyFlags 0x{old_flags:02X} -> 0x{key_flags:02X} "
                f"({web_key_flags_text(key_flags)}) age={age_ms}ms"
            )
            self._webKeyEventLogs.append(line)
            print(line, flush=True)
            self._lastWebKeyFlags = key_flags


    class PreviewCanvas(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.setMinimumHeight(260)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.raw_enc = [0, 0, 0]
            self.cam_yaw = -45.0
            self.cam_pitch = 28.0
            self.cam_scale = 2.2
            self.dragging = False
            self.last_mouse = QPoint()

            self.titleFont = QFont("Segoe UI", 10)
            self.titleFont.setStyleHint(QFont.SansSerif)
            self.axisFont = QFont("Segoe UI", 9)
            self.axisFont.setStyleHint(QFont.SansSerif)
            qconfig.themeChangedFinished.connect(self._refreshTheme)

        def setEncoderValues(self, values: Sequence[int]) -> None:
            self.raw_enc = [int(v) for v in values[:3]]
            self.update()

        def _refreshTheme(self, *_args) -> None:
            self.update()

        def _scenePalette(self) -> Dict[str, QColor]:
            if isDarkTheme():
                return {
                    "backgroundTop": QColor("#1A2028"),
                    "backgroundBottom": QColor("#11161D"),
                    "border": QColor(255, 255, 255, 30),
                    "gridMinor": QColor("#2E3B49"),
                    "gridMajor": QColor("#4E6174"),
                    "axisX": QColor("#FF8F83"),
                    "axisY": QColor("#52E2B8"),
                    "axisZ": QColor("#72B8FF"),
                    "axisLabel": QColor("#B7C4D2"),
                    "link0": QColor("#9AA5B4"),
                    "link1": QColor("#F0C45C"),
                    "link2": QColor("#78BCFF"),
                    "endEffector": QColor("#7DE7A8"),
                    "caption": QColor("#DCE6F0"),
                }

            return {
                "backgroundTop": QColor("#F7FAFD"),
                "backgroundBottom": QColor("#E9F0F7"),
                "border": QColor(80, 104, 128, 42),
                "gridMinor": QColor("#D6E0EA"),
                "gridMajor": QColor("#A9B9CA"),
                "axisX": QColor("#FF7B72"),
                "axisY": QColor("#3DD9B4"),
                "axisZ": QColor("#4DA3FF"),
                "axisLabel": QColor("#5C6F82"),
                "link0": QColor("#BFC8D6"),
                "link1": QColor("#F7C65C"),
                "link2": QColor("#75B7FF"),
                "endEffector": QColor("#90F0A1"),
                "caption": QColor("#43586E"),
            }

        def mousePressEvent(self, event) -> None:  # type: ignore[override]
            if event.button() == Qt.LeftButton:
                self.dragging = True
                self.last_mouse = event.pos()
                event.accept()
                return
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
            if self.dragging:
                delta = event.pos() - self.last_mouse
                self.last_mouse = event.pos()
                self.cam_yaw -= delta.x() * 0.45
                self.cam_pitch += delta.y() * 0.35
                self.cam_pitch = max(-85.0, min(85.0, self.cam_pitch))
                self.update()
                event.accept()
                return
            super().mouseMoveEvent(event)

        def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
            if event.button() == Qt.LeftButton:
                self.dragging = False
            super().mouseReleaseEvent(event)

        def wheelEvent(self, event) -> None:  # type: ignore[override]
            delta = event.angleDelta().y()
            if delta:
                self.cam_scale *= 1.0 + (0.08 if delta > 0 else -0.08)
                self.cam_scale = max(1.1, min(5.5, self.cam_scale))
                self.update()
                event.accept()
                return
            super().wheelEvent(event)

        def _fk_points(self) -> List[Tuple[float, float, float]]:
            theta = [math.radians(calibrated_raw_to_deg(self.raw_enc[i])) for i in range(3)]
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

        def _project(self, point: Tuple[float, float, float]) -> QPointF:
            x, y, z = point
            yaw = math.radians(self.cam_yaw)
            pitch = math.radians(self.cam_pitch)

            x1 = x * math.cos(yaw) + y * math.sin(yaw)
            y1 = -x * math.sin(yaw) + y * math.cos(yaw)
            y2 = z * math.cos(pitch) - y1 * math.sin(pitch)

            width = max(1.0, float(self.width()))
            height = max(1.0, float(self.height()))
            cx = width * 0.5
            cy = height * 0.58
            return QPointF(cx + x1 * self.cam_scale, cy - y2 * self.cam_scale)

        def paintEvent(self, event) -> None:  # type: ignore[override]
            _ = event
            painter = QPainter(self)
            painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
            palette = self._scenePalette()

            rect = QRectF(self.rect()).adjusted(4.0, 4.0, -4.0, -4.0)
            path = QPainterPath()
            path.addRoundedRect(rect, 18.0, 18.0)
            painter.setClipPath(path)

            gradient = QLinearGradient(rect.topLeft(), rect.bottomLeft())
            gradient.setColorAt(0.0, palette["backgroundTop"])
            gradient.setColorAt(1.0, palette["backgroundBottom"])
            painter.fillRect(rect, gradient)

            painter.setPen(QPen(palette["border"], 1.0))
            painter.drawRoundedRect(rect, 18.0, 18.0)

            grid_range = 220
            step = 20
            for value in range(-grid_range, grid_range + 1, step):
                p1 = self._project((value, -grid_range, 0.0))
                p2 = self._project((value, grid_range, 0.0))
                p3 = self._project((-grid_range, value, 0.0))
                p4 = self._project((grid_range, value, 0.0))
                color = palette["gridMinor"] if value != 0 else palette["gridMajor"]
                painter.setPen(QPen(color, 1.0))
                painter.drawLine(p1, p2)
                painter.drawLine(p3, p4)

            origin = self._project((0.0, 0.0, 0.0))
            x_tip = self._project((120.0, 0.0, 0.0))
            y_tip = self._project((0.0, 120.0, 0.0))
            z_tip = self._project((0.0, 0.0, 120.0))

            painter.setPen(QPen(palette["axisX"], 2.2))
            painter.drawLine(origin, x_tip)
            painter.setPen(QPen(palette["axisY"], 2.2))
            painter.drawLine(origin, y_tip)
            painter.setPen(QPen(palette["axisZ"], 2.2))
            painter.drawLine(origin, z_tip)

            painter.setFont(self.axisFont)
            painter.setPen(palette["axisLabel"])
            painter.drawText(QPointF(x_tip.x() + 8, x_tip.y()), "X")
            painter.drawText(QPointF(y_tip.x(), y_tip.y() - 8), "Y")
            painter.drawText(QPointF(z_tip.x() + 8, z_tip.y()), "Z")

            points = self._fk_points()
            projected = [self._project(p) for p in points]
            colors = [palette["link0"], palette["link1"], palette["link2"]]
            for idx in range(3):
                painter.setPen(QPen(colors[idx], 7.0, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
                painter.drawLine(projected[idx], projected[idx + 1])
                painter.setPen(Qt.NoPen)
                painter.setBrush(colors[idx])
                painter.drawEllipse(projected[idx], 5.0, 5.0)

            painter.setBrush(palette["endEffector"])
            painter.drawEllipse(projected[-1], 7.0, 7.0)

            painter.setPen(palette["caption"])
            painter.setFont(self.titleFont)
            painter.drawText(
                QRectF(18.0, 16.0, rect.width() - 36.0, 24.0),
                Qt.AlignLeft | Qt.AlignVCenter,
                "控制器三维预览  拖动旋转  |  滚轮缩放",
            )


    class EncoderRingCard(CardWidget):
        def __init__(self, index: int, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.index = index
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            layout = QHBoxLayout(self)
            layout.setContentsMargins(14, 12, 14, 12)
            layout.setSpacing(14)

            self.ring = ProgressRing(self)
            self.ring.setRange(ENCODER_MIN, ENCODER_MAX)
            self.ring.setValue(0)
            self.ring.setFixedSize(92, 92)
            self.ring.setTextVisible(False)
            self.ring.setUseAni(False)
            self.ring.setStrokeWidth(9)
            layout.addWidget(self.ring, 0, Qt.AlignVCenter)

            infoLayout = QVBoxLayout()
            infoLayout.setContentsMargins(0, 0, 0, 0)
            infoLayout.setSpacing(4)

            headerLayout = QHBoxLayout()
            headerLayout.setContentsMargins(0, 0, 0, 0)
            headerLayout.setSpacing(6)

            self.iconLabel = QLabel(self)
            self.titleLabel = BodyLabel(f"编码器 {index}", self)
            self.titleLabel.setStyleSheet("font-weight: 600;")
            headerLayout.addWidget(self.iconLabel, 0, Qt.AlignLeft | Qt.AlignVCenter)
            headerLayout.addWidget(self.titleLabel, 0, Qt.AlignLeft | Qt.AlignVCenter)
            headerLayout.addStretch(1)
            infoLayout.addLayout(headerLayout)

            self.valueLabel = SubtitleLabel("--", self)
            self.valueLabel.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.angleLabel = CaptionLabel("--", self)
            self.angleLabel.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            infoLayout.addWidget(self.valueLabel)
            infoLayout.addWidget(self.angleLabel)
            infoLayout.addStretch(1)

            layout.addLayout(infoLayout, 1)
            qconfig.themeChangedFinished.connect(self._applyTheme)
            self._applyTheme()

        def updateValue(self, raw: int, enabled: bool) -> None:
            if not enabled:
                self.ring.setValue(0)
                self.valueLabel.setText("--")
                self.angleLabel.setText("--")
                return
            self.ring.setValue(raw)
            self.valueLabel.setText(f"{raw}")
            self.angleLabel.setText(f"{calibrated_raw_to_deg(raw):+.2f} deg")

        def _applyTheme(self, *_args) -> None:
            color = QColor("#DCE6F0") if isDarkTheme() else QColor("#43586E")
            self.iconLabel.setPixmap(themed_icon_pixmap(FIF.ROTATE, 16, color))


    class PreviewPage(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.setObjectName("preview-page")

            content = build_scrollable_page(self)
            root = QHBoxLayout(content)
            root.setContentsMargins(24, 20, 24, 24)
            root.setSpacing(16)

            leftCard = CompactGroupCard(FIF.VIEW, "3D 预览", "", self)
            self.canvas = PreviewCanvas(leftCard)
            leftCard.addWidget(self.canvas, 1)
            root.addWidget(leftCard, 10)

            rightLayout = QVBoxLayout()
            rightLayout.setSpacing(10)
            self.encoderCards = [EncoderRingCard(index, self) for index in range(1, 4)]
            for card in self.encoderCards:
                rightLayout.addWidget(card)
            rightLayout.addStretch(1)
            root.addLayout(rightLayout, 4)

        def updateView(self, frame: Optional[DecodedFrame]) -> None:
            values = frame.enc_raw if frame is not None else (0, 0, 0)
            self.canvas.setEncoderValues(values)
            for index, card in enumerate(self.encoderCards):
                card.updateValue(values[index], frame is not None)


    class MonitorPage(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.setObjectName("monitor-page")
            self._treeGuard = False
            self._latestValues: Dict[str, str] = {}

            root = QVBoxLayout(self)
            root.setContentsMargins(24, 20, 24, 24)
            root.setSpacing(14)

            self.payloadCard = CompactGroupCard(FIF.CODE, "原始载荷", "", self)
            self.payloadValue = BodyLabel("等待串口数据…", self.payloadCard)
            self.payloadValue.setWordWrap(True)
            mono = QFont("Consolas")
            mono.setStyleHint(QFont.Monospace)
            self.payloadValue.setFont(mono)
            self.payloadCard.addWidget(self.payloadValue)
            root.addWidget(self.payloadCard)

            split = QHBoxLayout()
            split.setContentsMargins(0, 0, 0, 0)
            split.setSpacing(14)
            root.addLayout(split, 1)

            treeCard = CompactGroupCard(None, "勾选监看的字段", "", self)
            self.tree = TreeWidget(treeCard)
            self.tree.setColumnCount(2)
            self.tree.setHeaderLabels(["监看项", "说明"])
            self.tree.setUniformRowHeights(True)
            self.tree.setAnimated(True)
            self.tree.header().setStretchLastSection(False)
            self.tree.header().setSectionResizeMode(0, QHeaderView.ResizeToContents)
            self.tree.header().setSectionResizeMode(1, QHeaderView.Stretch)
            self.tree.itemChanged.connect(self._onTreeItemChanged)
            treeCard.addWidget(self.tree, 1)
            split.addWidget(treeCard, 7)

            tableCard = CompactGroupCard(None, "预览数据", "", self)
            self.table = TableWidget(tableCard)
            self.table.setColumnCount(4)
            self.table.setHorizontalHeaderLabels(["分类", "字段", "当前值", "说明"])
            self.table.setWordWrap(False)
            self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
            self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
            self.table.setAlternatingRowColors(True)
            self.table.setBorderVisible(True)
            self.table.setBorderRadius(8)
            self.table.verticalHeader().setVisible(False)
            self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
            self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
            self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
            self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
            tableCard.addWidget(self.table, 1)
            split.addWidget(tableCard, 8)

            self._buildTree()

        def _buildTree(self) -> None:
            self.tree.clear()

            def make_item(node: dict) -> QTreeWidgetItem:
                item = QTreeWidgetItem([node["label"], node["description"]])
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                item.setCheckState(0, Qt.Unchecked)

                key = node.get("key")
                if key is not None:
                    item.setData(0, Qt.UserRole, key)

                for child in node.get("children", []):
                    item.addChild(make_item(child))

                if item.childCount():
                    self._syncParentState(item)
                return item

            self._treeGuard = True
            for node in MONITOR_TREE:
                self.tree.addTopLevelItem(make_item(node))
            self._treeGuard = False
            self.tree.expandToDepth(1)
            self._refreshTable()

        def _syncParentState(self, item: QTreeWidgetItem) -> None:
            if item.childCount() == 0:
                return

            checked = 0
            partial = False
            for index in range(item.childCount()):
                state = item.child(index).checkState(0)
                if state == Qt.PartiallyChecked:
                    partial = True
                elif state == Qt.Checked:
                    checked += 1

            if partial or (0 < checked < item.childCount()):
                item.setCheckState(0, Qt.PartiallyChecked)
            elif checked == item.childCount():
                item.setCheckState(0, Qt.Checked)
            else:
                item.setCheckState(0, Qt.Unchecked)

        def _setChildrenState(self, item: QTreeWidgetItem, state: int) -> None:
            for index in range(item.childCount()):
                child = item.child(index)
                child.setCheckState(0, state)
                self._setChildrenState(child, state)

        def _updateAncestors(self, item: Optional[QTreeWidgetItem]) -> None:
            while item is not None:
                self._syncParentState(item)
                item = item.parent()

        def _onTreeItemChanged(self, item: QTreeWidgetItem, column: int) -> None:
            _ = column
            if self._treeGuard:
                return

            self._treeGuard = True
            state = item.checkState(0)
            if item.childCount() and state in (Qt.Checked, Qt.Unchecked):
                self._setChildrenState(item, state)
            self._updateAncestors(item.parent())
            self._treeGuard = False
            self._refreshTable()

        def _selectedKeys(self) -> List[str]:
            keys: List[str] = []

            def visit(item: QTreeWidgetItem) -> None:
                key = item.data(0, Qt.UserRole)
                if key and item.checkState(0) == Qt.Checked and item.childCount() == 0:
                    keys.append(str(key))
                for index in range(item.childCount()):
                    visit(item.child(index))

            for index in range(self.tree.topLevelItemCount()):
                visit(self.tree.topLevelItem(index))
            return keys

        def _refreshTable(self) -> None:
            keys = self._selectedKeys()
            self.table.setRowCount(len(keys))

            for row, key in enumerate(keys):
                category, label, description = FIELD_META.get(key, ("其它", key, ""))
                value = self._latestValues.get(key, "--")
                cells = [category, label, value, description]
                for column, text in enumerate(cells):
                    item = QTableWidgetItem(text)
                    item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                    self.table.setItem(row, column, item)

        def updateView(
            self,
            frame: Optional[DecodedFrame],
            stats: Dict[str, int],
            rate_hz: float,
            connected: bool,
            error: str,
        ) -> None:
            self._latestValues = build_monitor_values(frame, stats, rate_hz, connected, error)
            self.payloadValue.setText(frame.payload_hex if frame is not None else "等待串口数据…")
            self._refreshTable()


    class MidiTrackListDelegate(QStyledItemDelegate):
        def paint(self, painter: QPainter, option: QStyleOptionViewItem, index) -> None:  # type: ignore[override]
            painter.save()
            painter.setRenderHint(QPainter.Antialiasing)

            selected = bool(option.state & QStyle.State_Selected)
            hovered = bool(option.state & QStyle.State_MouseOver)
            outer_rect = option.rect.adjusted(6, 3, -6, -3)

            if isDarkTheme():
                base = QColor(255, 255, 255, 14)
                hover = QColor(255, 255, 255, 20)
                selected_fill = QColor(255, 255, 255, 28)
                border = QColor(255, 255, 255, 22)
                selected_border = QColor(255, 255, 255, 42)
                badge_fill = QColor(255, 255, 255, 24)
                badge_border = QColor(255, 255, 255, 34)
            else:
                base = QColor(0, 0, 0, 8)
                hover = QColor(0, 0, 0, 12)
                selected_fill = QColor(0, 0, 0, 16)
                border = QColor(0, 0, 0, 18)
                selected_border = QColor(0, 0, 0, 28)
                badge_fill = QColor(255, 255, 255, 224)
                badge_border = QColor(0, 0, 0, 18)

            fill = selected_fill if selected else hover if hovered else base
            outline = selected_border if selected else border
            painter.setPen(QPen(outline, 1))
            painter.setBrush(fill)
            painter.drawRoundedRect(outer_rect, 10, 10)

            badge_rect = QRect(outer_rect.left() + 12, outer_rect.top() + (outer_rect.height() - 40) // 2, 40, 40)
            painter.setPen(QPen(badge_border, 1))
            painter.setBrush(badge_fill)
            painter.drawRoundedRect(badge_rect, 10, 10)

            emoji = str(index.data(TRACK_EMOJI_ROLE) or "🎵")
            emoji_font = QFont(option.font)
            emoji_font.setPointSize(max(12, emoji_font.pointSize() + 2))
            painter.setFont(emoji_font)
            painter.setPen(option.palette.text().color())
            painter.drawText(badge_rect, Qt.AlignCenter, emoji)

            text_rect = outer_rect.adjusted(66, 10, -12, -10)
            title = str(index.data(TRACK_TITLE_ROLE) or "")
            summary = str(index.data(TRACK_SUMMARY_ROLE) or "")
            title_color = option.palette.text().color()
            subtitle_color = QColor(title_color)
            subtitle_color.setAlpha(185 if isDarkTheme() else 165)

            title_font = QFont(option.font)
            title_font.setWeight(QFont.DemiBold)
            painter.setFont(title_font)
            title_metrics = painter.fontMetrics()
            title_text = title_metrics.elidedText(title, Qt.ElideRight, max(0, text_rect.width()))
            painter.setPen(title_color)
            painter.drawText(QRect(text_rect.left(), text_rect.top(), text_rect.width(), 20), Qt.AlignLeft | Qt.AlignVCenter, title_text)

            subtitle_font = QFont(option.font)
            subtitle_font.setPointSize(max(9, subtitle_font.pointSize() - 1))
            painter.setFont(subtitle_font)
            subtitle_metrics = painter.fontMetrics()
            subtitle_text = subtitle_metrics.elidedText(summary, Qt.ElideRight, max(0, text_rect.width()))
            painter.setPen(subtitle_color)
            painter.drawText(QRect(text_rect.left(), text_rect.bottom() - 18, text_rect.width(), 18), Qt.AlignLeft | Qt.AlignVCenter, subtitle_text)
            painter.restore()

        def sizeHint(self, option: QStyleOptionViewItem, index) -> QSize:  # type: ignore[override]
            return QSize(0, 78)


    class MidiWorkbenchPane(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.mainLayout = QVBoxLayout(self)
            self.mainLayout.setContentsMargins(0, 0, 0, 0)
            self.mainLayout.setSpacing(0)

            self.tableCard = CompactGroupCard(None, "轨道音符", "", self)
            actions = QHBoxLayout()
            actions.setContentsMargins(0, 0, 0, 0)
            actions.setSpacing(8)

            self.summaryLabel = CaptionLabel("未选择轨道", self.tableCard)
            self.deleteButton = PushButton("删除选中音符", self.tableCard)
            self.deleteButton.setIcon(FIF.DELETE)
            self.deleteButton.setEnabled(False)

            actions.addWidget(self.summaryLabel, 1)
            actions.addWidget(self.deleteButton, 0, Qt.AlignRight)
            self.tableCard.addLayout(actions)

            self.noteTable = TableWidget(self.tableCard)
            self.noteTable.setColumnCount(5)
            self.noteTable.setHorizontalHeaderLabels(["序号", "时长", "开始时间", "音调", "频率"])
            self.noteTable.setWordWrap(False)
            self.noteTable.setEditTriggers(QAbstractItemView.NoEditTriggers)
            self.noteTable.setSelectionBehavior(QAbstractItemView.SelectRows)
            self.noteTable.setSelectionMode(QAbstractItemView.SingleSelection)
            self.noteTable.setAlternatingRowColors(True)
            self.noteTable.setBorderVisible(True)
            self.noteTable.setBorderRadius(8)
            self.noteTable.verticalHeader().setVisible(False)
            self.noteTable.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
            self.noteTable.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
            self.noteTable.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
            self.noteTable.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
            self.noteTable.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
            self.tableCard.addWidget(self.noteTable, 1)
            self.mainLayout.addWidget(self.tableCard, 1)


    class MidiImportPage(QWidget):
        def __init__(self, parent: Optional[QWidget] = None) -> None:
            super().__init__(parent)
            self.setObjectName("midi-page")

            self.midiTracks: List[MelodyTrack] = []
            self.jsonTracks: List[MelodyTrack] = []
            self.trackMap: Dict[str, MelodyTrack] = {}
            self.currentTrackId: Optional[str] = None
            self.currentMidiPath: Optional[Path] = None
            self.currentJsonPath: Optional[Path] = None
            self.lastUploadPackage: Optional[MelodyUploadPackage] = None

            content = build_scrollable_page(self)
            root = QHBoxLayout(content)
            root.setContentsMargins(24, 20, 24, 24)
            root.setSpacing(16)
            leftLayout = QVBoxLayout()
            leftLayout.setSpacing(12)
            rightLayout = QVBoxLayout()
            rightLayout.setSpacing(12)
            root.addLayout(leftLayout, 5)
            root.addLayout(rightLayout, 8)

            self.importCard = CompactGroupCard(FIF.DOWNLOAD, "导入", "", self)
            self.sourceFileLabel = BodyLabel("未导入MIDI文件", self.importCard)
            self.sourceFileLabel.setWordWrap(False)
            self.importCard.addWidget(self.sourceFileLabel)

            buttonRow = QHBoxLayout()
            buttonRow.setContentsMargins(0, 2, 0, 0)
            buttonRow.setSpacing(8)
            self.importMidiButton = PrimaryPushButton("导入 MIDI", self.importCard)
            self.importMidiButton.setIcon(FIF.MUSIC)
            self.importJsonButton = PushButton("导入 JSON", self.importCard)
            self.importJsonButton.setIcon(FIF.DOCUMENT)
            buttonRow.addWidget(self.importMidiButton, 1)
            buttonRow.addWidget(self.importJsonButton, 1)
            self.importCard.addLayout(buttonRow)
            leftLayout.addWidget(self.importCard)

            self.trackCard = CompactGroupCard(FIF.MUSIC, "轨道解析", "", self)
            self.trackList = ListWidget(self.trackCard)
            self.trackList.setFrameShape(QFrame.NoFrame)
            self.trackList.setSelectionMode(QAbstractItemView.SingleSelection)
            self.trackList.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
            self.trackList.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.trackList.setWordWrap(True)
            self.trackList.setIconSize(QSize(28, 28))
            self.trackList.setUniformItemSizes(False)
            self.trackCard.addWidget(self.trackList, 1)
            leftLayout.addWidget(self.trackCard, 1)

            self.editorPane = MidiWorkbenchPane(self)
            rightLayout.addWidget(self.editorPane, 1)

            self.actionBar = CardWidget(self)
            actionLayout = QHBoxLayout(self.actionBar)
            actionLayout.setContentsMargins(14, 12, 14, 12)
            actionLayout.setSpacing(10)

            actionLayout.addWidget(BodyLabel("BPM", self.actionBar))
            self.bpmSpin = SpinBox(self.actionBar)
            self.bpmSpin.setRange(20, 300)
            self.bpmSpin.setValue(120)
            self.bpmSpin.setFixedWidth(132)
            actionLayout.addWidget(self.bpmSpin)

            actionLayout.addWidget(BodyLabel("音符间隙", self.actionBar))
            self.gapSpin = SpinBox(self.actionBar)
            self.gapSpin.setRange(0, 500)
            self.gapSpin.setValue(30)
            self.gapSpin.setSuffix(" ms")
            self.gapSpin.setFixedWidth(132)
            actionLayout.addWidget(self.gapSpin)

            actionLayout.addStretch(1)
            self.exportButton = PushButton("导出", self.actionBar)
            self.exportButton.setIcon(FIF.SAVE)
            self.uploadButton = PrimaryPushButton("上传", self.actionBar)
            self.uploadButton.setIcon(FIF.SEND)
            actionLayout.addWidget(self.exportButton)
            actionLayout.addWidget(self.uploadButton)
            rightLayout.addWidget(self.actionBar, 0)

            self.importMidiButton.clicked.connect(self._importMidiFile)
            self.importJsonButton.clicked.connect(self._importJsonModule)
            self.trackList.currentRowChanged.connect(self._onTrackSelectionChanged)
            self.editorPane.deleteButton.clicked.connect(self._deleteSelectedNote)
            self.exportButton.clicked.connect(self._exportCurrentTrack)
            self.uploadButton.clicked.connect(self._uploadCurrentTrack)
            self.bpmSpin.valueChanged.connect(self._refreshNoteTable)
            self.gapSpin.valueChanged.connect(self._refreshExportState)
            self.editorPane.noteTable.itemSelectionChanged.connect(self._refreshExportState)

            self._refreshTrackList()
            self._refreshNoteTable()

        def _showInfo(self, icon, title: str, content: str) -> None:
            window = self.window()
            parent = window if isinstance(window, QWidget) else self
            InfoBar.new(
                icon=icon,
                title=title,
                content=content,
                orient=Qt.Horizontal,
                isClosable=True,
                duration=2600,
                position=InfoBarPosition.TOP_RIGHT,
                parent=parent,
            )

        def _allTracks(self) -> List[MelodyTrack]:
            return [*self.midiTracks, *self.jsonTracks]

        def _setSourceDisplay(self, text: str) -> None:
            self.sourceFileLabel.setText(text)
            self.sourceFileLabel.setToolTip(text)

        def _refreshTrackList(self) -> None:
            current_id = self.currentTrackId
            self.trackMap = {track.track_id: track for track in self._allTracks()}
            self.trackList.clear()

            for track in self._allTracks():
                item = QListWidgetItem(emoji_icon(track.emoji), f"{track.name}\n{melody_track_summary(track)}")
                item.setData(TRACK_ID_ROLE, track.track_id)
                item.setToolTip(track.name)
                item.setSizeHint(QSize(0, 56))
                self.trackList.addItem(item)

            if not self.trackMap:
                self.currentTrackId = None
                self.trackList.setCurrentRow(-1)
                return

            target_row = 0
            if current_id is not None:
                for row in range(self.trackList.count()):
                    item = self.trackList.item(row)
                    if item.data(TRACK_ID_ROLE) == current_id:
                        target_row = row
                        break

            self.trackList.setCurrentRow(target_row)

        def _selectedTrack(self) -> Optional[MelodyTrack]:
            if self.currentTrackId is None:
                return None
            return self.trackMap.get(self.currentTrackId)

        def _onTrackSelectionChanged(self, row: int) -> None:
            self.currentTrackId = None
            if row >= 0:
                item = self.trackList.item(row)
                if item is not None:
                    self.currentTrackId = item.data(TRACK_ID_ROLE)
            self._refreshNoteTable()

        def _importMidiFile(self) -> None:
            if mido is None:
                self._showInfo(InfoBarIcon.ERROR, "缺少依赖", "请先安装 mido 后再导入 MIDI")
                return

            file_name, _ = QFileDialog.getOpenFileName(
                self,
                "选择 MIDI 文件",
                str(Path.cwd()),
                "MIDI 文件 (*.mid *.midi)",
            )
            if not file_name:
                return

            midi_path = Path(file_name)
            try:
                tracks, bpm = extract_melody_tracks(midi_path)
            except Exception as exc:
                self._showInfo(InfoBarIcon.ERROR, "导入失败", str(exc))
                return

            self.currentMidiPath = midi_path
            self.midiTracks = tracks
            self.currentJsonPath = None
            self.jsonTracks = []
            self._setSourceDisplay(midi_path.name)
            self.gapSpin.setValue(30)

            if tracks:
                self.bpmSpin.setValue(bpm)
                self.currentTrackId = tracks[0].track_id
                self._showInfo(InfoBarIcon.SUCCESS, "MIDI 已导入", f"解析到 {len(tracks)} 个可编辑轨道")
            else:
                self.currentTrackId = None
                self._showInfo(InfoBarIcon.WARNING, "没有可用轨道", "文件里没有检测到有效的乐器音符轨道")

            self._refreshTrackList()
            self._refreshNoteTable()

        def _importJsonModule(self) -> None:
            file_name, _ = QFileDialog.getOpenFileName(
                self,
                "选择 JSON 音频",
                str(Path.cwd()),
                "JSON 文件 (*.json)",
            )
            if not file_name:
                return

            json_path = Path(file_name)
            try:
                track, bpm, gap_ms = load_melody_module(json_path)
            except Exception as exc:
                self._showInfo(InfoBarIcon.ERROR, "音频导入失败", str(exc))
                return

            self.currentJsonPath = json_path
            self.jsonTracks = [track]
            self.currentMidiPath = None
            self.midiTracks = []
            self._setSourceDisplay(json_path.name)
            self.bpmSpin.setValue(bpm)
            self.gapSpin.setValue(gap_ms)
            self.currentTrackId = track.track_id
            self._refreshTrackList()
            self._refreshNoteTable()
            self._showInfo(InfoBarIcon.SUCCESS, "JSON 音频已导入", f"已载入音频：{track.name}")

        def _refreshNoteTable(self) -> None:
            track = self._selectedTrack()
            bpm = int(self.bpmSpin.value())
            table = self.editorPane.noteTable

            if track is None:
                self.editorPane.summaryLabel.setText("未选择轨道")
                table.setRowCount(0)
                self._refreshExportState()
                return

            notes = sorted(track.notes, key=lambda item: (item.start_beats, item.source_order, item.midi_note))
            table.setRowCount(len(notes))
            for row, note in enumerate(notes):
                start_ms = beats_to_ms(note.start_beats, bpm)
                duration_ms = beats_to_ms(note.duration_beats, bpm)
                cells = [
                    str(row + 1),
                    format_note_time(duration_ms),
                    format_note_time(start_ms),
                    midi_note_name(note.midi_note),
                    f"{midi_note_to_freq_hz(note.midi_note)} Hz",
                ]
                for column, text in enumerate(cells):
                    item = QTableWidgetItem(text)
                    item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                    table.setItem(row, column, item)

            source_hint = track.source_name
            self.editorPane.summaryLabel.setText(
                f"{track.name} · {len(notes)} 条 · {source_hint}"
            )
            self._refreshExportState()

        def _refreshExportState(self) -> None:
            track = self._selectedTrack()
            has_track = track is not None and bool(track.notes)
            has_selection = self.editorPane.noteTable.currentRow() >= 0
            self.editorPane.deleteButton.setEnabled(has_track and has_selection)
            self.exportButton.setEnabled(has_track)
            self.uploadButton.setEnabled(has_track)

        def _deleteSelectedNote(self) -> None:
            track = self._selectedTrack()
            row = self.editorPane.noteTable.currentRow()
            if track is None or row < 0 or row >= len(track.notes):
                return

            del track.notes[row]
            _refresh_track_counters(track)
            self._refreshTrackList()
            self._refreshNoteTable()

        def _exportCurrentTrack(self) -> None:
            track = self._selectedTrack()
            if track is None:
                self._showInfo(InfoBarIcon.WARNING, "没有可导出的轨道", "请先选择一个轨道")
                return

            default_name = "".join(ch if ch.isalnum() or ch in ("-", "_", " ") else "_" for ch in track.name).strip() or "melody"
            default_path = Path.cwd() / f"{default_name}.json"
            file_name, _ = QFileDialog.getSaveFileName(
                self,
                "导出 JSON 音频",
                str(default_path),
                "JSON 文件 (*.json)",
            )
            if not file_name:
                return

            package = build_upload_package(track, int(self.bpmSpin.value()), int(self.gapSpin.value()))
            document = melody_module_document(track, int(self.bpmSpin.value()), int(self.gapSpin.value()))
            out_path = Path(file_name)
            out_path.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
            self.lastUploadPackage = package
            self._showInfo(
                InfoBarIcon.SUCCESS,
                "导出完成",
                f"{out_path.name} · {len(package.steps)} 个 BuzzerPwmStep",
            )

        def _uploadCurrentTrack(self) -> None:
            track = self._selectedTrack()
            if track is None:
                self._showInfo(InfoBarIcon.WARNING, "没有可上传的轨道", "请先选择一个轨道")
                return

            package = build_upload_package(track, int(self.bpmSpin.value()), int(self.gapSpin.value()))
            self.lastUploadPackage = package
            self._showInfo(
                InfoBarIcon.INFORMATION,
                "敬请期待",
                f"已准备好上传数据包，包含 {len(package.steps)} 个 BuzzerPwmStep，但上传功能尚未实现",
            )


    class PreviewMonitorWindow(FluentWindow):
        def __init__(self, port: str, baud: int) -> None:
            super().__init__()

            self.worker: Optional[SerialWorker] = None
            self.webServiceManager = WebServiceManager()
            self.xrBridgeManager = XrUartBridgeManager()
            self.last_frame: Optional[DecodedFrame] = None
            self.last_stats: Dict[str, int] = {"good": 0, "total": 0, "bad_header_crc": 0, "bad_frame_crc": 0, "other": 0}
            self.last_rate_hz = 0.0
            self.last_connected = False
            self.last_error = ""
            self.last_xr_status = XrDeviceStatus()
            self.last_xr_lines: List[str] = []
            self.last_xr_bridge = XrBridgeSnapshot()
            self.lastOutputInterface: Optional[str] = None
            self.lastStatusSignature: Optional[Tuple[str, str, str]] = None
            self.connectionStatusItem: Optional[ConnectionStatusNavItem] = None
            self.themeToggleButton: Optional[NavigationPushButton] = None
            self.githubButton: Optional[NavigationPushButton] = None
            self.connectTeachingTip = None

            self.overviewPage = OverviewPage(port, baud, self)
            self.xrControlPage = XrControlPage(self)
            self.previewPage = PreviewPage(self)
            self.monitorPage = MonitorPage(self)
            self.midiPage = MidiImportPage(self)

            self._initWindow()
            self._initNavigation()
            self._wireSignals()

            self.refreshTimer = QTimer(self)
            self.refreshTimer.timeout.connect(self._refresh)
            self.refreshTimer.start(33)
            self._refresh()

        def _initWindow(self) -> None:
            setTheme(Theme.AUTO)
            setThemeColor(DEFAULT_THEME_COLOR)

            self.resize(*DEFAULT_WINDOW_SIZE)
            self.setMinimumSize(*DEFAULT_MINIMUM_SIZE)
            self.setResizeEnabled(True)
            self.setWindowTitle("CCtrl 串口监视器")
            self.setWindowIcon(QIcon(":/qfluentwidgets/images/logo.png"))
            self.navigationInterface.setExpandWidth(NAVIGATION_EXPANDED_WIDTH)
            self.navigationInterface.setMinimumExpandWidth(0)
            self.navigationInterface.setReturnButtonVisible(False)

            screen = QApplication.primaryScreen()
            if screen is not None:
                geo = screen.availableGeometry()
                self.move(geo.center().x() - self.width() // 2, geo.center().y() - self.height() // 2)

        def _initNavigation(self) -> None:
            self.addSubInterface(self.xrControlPage, FIF.SEND, "XR-UART")
            self.addSubInterface(self.overviewPage, FIF.HOME, "概览")
            self.addSubInterface(self.previewPage, FIF.VIEW, "3D预览")
            self.addSubInterface(self.midiPage, FIF.MUSIC, "MIDI导入")
            self.addSubInterface(self.monitorPage, FIF.DOCUMENT, "载荷监视器")

            self.connectionStatusItem = ConnectionStatusNavItem(self.navigationInterface)
            self.navigationInterface.addWidget(
                "connection-status",
                self.connectionStatusItem,
                self._navigateToConnectionCard,
                NavigationItemPosition.BOTTOM,
                "查看连接状态并跳转到串口连接卡片",
            )

            self.navigationInterface.addSeparator(NavigationItemPosition.BOTTOM)

            self.themeToggleButton = NavigationPushButton(FIF.BRIGHTNESS, "", False, self.navigationInterface)
            self.navigationInterface.addWidget(
                "theme-toggle",
                self.themeToggleButton,
                self._cycleThemeMode,
                NavigationItemPosition.BOTTOM,
                "切换主题模式",
            )

            self.githubButton = NavigationPushButton(FIF.GITHUB, "项目仓库", False, self.navigationInterface)
            self.navigationInterface.addWidget(
                "github-link",
                self.githubButton,
                self._openProjectRepository,
                NavigationItemPosition.BOTTOM,
                PROJECT_GITHUB_URL,
            )

            self.navigationInterface.expand(useAni=False)
            self._updateConnectionStatusItem()
            self._applyThemeState()

        def _wireSignals(self) -> None:
            self.overviewPage.refreshPortsButton.clicked.connect(self.overviewPage.refreshPorts)
            self.overviewPage.connectButton.clicked.connect(self._connectSerial)
            self.overviewPage.disconnectButton.clicked.connect(self._disconnectSerial)
            self.xrControlPage.refreshBridgePortsButton.clicked.connect(self.xrControlPage.refreshBridgePorts)
            self.xrControlPage.refreshXrStatusButton.clicked.connect(self._requestXrStatus)
            self.xrControlPage.enterXrButton.clicked.connect(self._enterXrMode)
            self.xrControlPage.exitXrButton.clicked.connect(self._exitXrMode)
            self.xrControlPage.generateCertButton.clicked.connect(self._generateWebCerts)
            self.xrControlPage.startRelayButton.clicked.connect(self._startWebRelay)
            self.xrControlPage.stopRelayButton.clicked.connect(self._stopWebRelay)
            self.xrControlPage.positionScaleSpin.valueChanged.connect(self._saveRuntimeConfigFromUi)
            self.xrControlPage.rotationScaleSpin.valueChanged.connect(self._saveRuntimeConfigFromUi)
            self.xrControlPage.bridgePortSpin.valueChanged.connect(self._saveRuntimeConfigFromUi)
            qconfig.themeChangedFinished.connect(self._applyThemeState)

        def _currentThemeMode(self) -> Theme:
            mode = getattr(qconfig.themeMode, "value", Theme.AUTO)
            return mode if isinstance(mode, Theme) else Theme.AUTO

        def _nextThemeMode(self, mode: Theme) -> Theme:
            return {
                Theme.AUTO: Theme.DARK,
                Theme.DARK: Theme.LIGHT,
                Theme.LIGHT: Theme.AUTO,
            }[mode]

        def _themeModeText(self, mode: Theme) -> str:
            return {
                Theme.AUTO: "跟随系统",
                Theme.DARK: "深色",
                Theme.LIGHT: "浅色",
            }[mode]

        def _updateThemeToggleButton(self) -> None:
            if self.themeToggleButton is None:
                return

            mode = self._currentThemeMode()
            next_mode = self._nextThemeMode(mode)
            self.themeToggleButton.setText(f"主题：{self._themeModeText(mode)}")
            self.themeToggleButton.setToolTip(
                f"当前为{self._themeModeText(mode)}，点击切换到{self._themeModeText(next_mode)}"
            )

        def _applyThemeState(self, *_args) -> None:
            self._updateThemeToggleButton()
            self._updateConnectionStatusItem()
            if self.githubButton is not None:
                self.githubButton.setToolTip(PROJECT_GITHUB_URL)

            self.titleBar.setStyle(QApplication.style())
            self.titleBar.update()
            self.navigationInterface.update()
            self.previewPage.canvas.update()

        def _cycleThemeMode(self) -> None:
            setTheme(self._nextThemeMode(self._currentThemeMode()))

        def _openProjectRepository(self) -> None:
            QDesktopServices.openUrl(QUrl(PROJECT_GITHUB_URL))

        def _updateConnectionStatusItem(self) -> None:
            if self.connectionStatusItem is not None:
                self.connectionStatusItem.setStatus(self.last_connected, self.lastOutputInterface)

        def _clearConnectionTeachingTip(self, *_args) -> None:
            self.connectTeachingTip = None

        def _closeConnectionTeachingTip(self) -> None:
            if self.connectTeachingTip is None:
                return

            try:
                self.connectTeachingTip.close()
            except RuntimeError:
                pass
            finally:
                self.connectTeachingTip = None

        def _showConnectionTeachingTip(self) -> None:
            if hasattr(self.overviewPage, "_scrollArea"):
                self.overviewPage._scrollArea.ensureWidgetVisible(self.overviewPage.connectCard, 0, 24)  # type: ignore[attr-defined]

            self._closeConnectionTeachingTip()

            if self.last_connected:
                content = "当前已连接。可在这里点击“断开”，或先断开后重新选择串口和波特率。"
            else:
                content = "请先选择串口和波特率，然后点击“连接”开始监视。"

            self.connectTeachingTip = TeachingTip.create(
                self.overviewPage.connectCard,
                "串口连接",
                content,
                icon=FIF.CONNECT,
                isClosable=True,
                duration=3200,
                tailPosition=TeachingTipTailPosition.BOTTOM_LEFT,
                parent=self,
            )
            self.connectTeachingTip.destroyed.connect(self._clearConnectionTeachingTip)

        def _showConnectionTeachingTipWhenStable(self, previous_pos: Optional[QPoint] = None, attempt: int = 0) -> None:
            current_pos = self.overviewPage.connectCard.mapToGlobal(QPoint())
            if previous_pos is not None and (current_pos - previous_pos).manhattanLength() <= 1:
                self._showConnectionTeachingTip()
                return

            if attempt >= 8:
                self._showConnectionTeachingTip()
                return

            if hasattr(self.overviewPage, "_scrollArea"):
                self.overviewPage._scrollArea.ensureWidgetVisible(self.overviewPage.connectCard, 0, 24)  # type: ignore[attr-defined]

            QTimer.singleShot(
                40,
                lambda pos=QPoint(current_pos), next_attempt=attempt + 1: self._showConnectionTeachingTipWhenStable(
                    pos, next_attempt
                ),
            )

        def _queueConnectionTeachingTip(self) -> None:
            if hasattr(self.overviewPage, "_scrollArea"):
                self.overviewPage._scrollArea.ensureWidgetVisible(self.overviewPage.connectCard, 0, 24)  # type: ignore[attr-defined]

            QTimer.singleShot(0, self._showConnectionTeachingTipWhenStable)

        def _navigateToConnectionCard(self) -> None:
            self.switchTo(self.overviewPage)
            self.navigationInterface.setCurrentItem(self.overviewPage.objectName())
            QTimer.singleShot(0, self._queueConnectionTeachingTip)

        def _showInfo(self, icon, title: str, content: str) -> None:
            InfoBar.new(
                icon=icon,
                title=title,
                content=content,
                orient=Qt.Horizontal,
                isClosable=True,
                duration=2600,
                position=InfoBarPosition.TOP_RIGHT,
                parent=self,
            )

        def _showInfoLater(self, icon, title: str, content: str) -> None:
            QTimer.singleShot(0, lambda: self._showInfo(icon, title, content))

        def _saveRuntimeConfigFromUi(self, *_args) -> None:
            save_runtime_config(self.xrControlPage.runtimeConfig())

        def _ensureXrBridgeRunning(self) -> Tuple[bool, str]:
            self._saveRuntimeConfigFromUi()
            config = load_runtime_config()
            port = self.xrControlPage.selectedBridgePort()
            baud = self.xrControlPage.selectedBridgeBaud()
            bridge_host = str(config.get("bridgeHost", "127.0.0.1") or "127.0.0.1")
            bridge_port = int(config.get("bridgePort", 8791) or 8791)
            return self.xrBridgeManager.start(port, baud, bridge_host, bridge_port)

        def _sendXrCommand(self, command: str, success_hint: str) -> None:
            ok, message = self._ensureXrBridgeRunning()
            if not ok:
                self._showInfo(InfoBarIcon.ERROR, "XR 串口桥未就绪", message)
                return

            if self.xrBridgeManager.send_command(command):
                self._showInfo(InfoBarIcon.INFORMATION, "命令已发送", success_hint)
                return

            self._showInfo(
                InfoBarIcon.ERROR,
                "命令发送失败",
                message or "请检查 ESP32 USB 串口是否被占用",
            )

        def _requestXrStatus(self) -> None:
            self._sendXrCommand("@XR STATUS", "正在查询 XR-UART 状态")

        def _enterXrMode(self) -> None:
            self._saveRuntimeConfigFromUi()
            ok, message = self._ensureXrBridgeRunning()
            if not ok:
                self._showInfo(InfoBarIcon.ERROR, "XR 串口桥启动失败", message)
                return
            if not self.webServiceManager.is_running():
                self._runWebTask("Web 服务启动", self.webServiceManager.start_service)
            self._sendXrCommand("@XR XR_ON", "正在切换到 XR-UART 模式")

        def _exitXrMode(self) -> None:
            self._sendXrCommand("@XR XR_OFF", "正在退出 XR-UART 模式并恢复节点链路")
            self.webServiceManager.stop_service()
            self.xrBridgeManager.stop()

        def _runWebTask(self, action_name: str, fn) -> None:
            def worker() -> None:
                ok, message = fn()
                icon = InfoBarIcon.SUCCESS if ok else InfoBarIcon.ERROR
                title = f"{action_name}{'完成' if ok else '失败'}"
                self._showInfoLater(icon, title, message.strip() or action_name)

            threading.Thread(target=worker, daemon=True).start()

        def _generateWebCerts(self) -> None:
            self._runWebTask("证书生成", self.webServiceManager.generate_certs)

        def _startWebRelay(self) -> None:
            self._saveRuntimeConfigFromUi()
            ok, message = self._ensureXrBridgeRunning()
            if not ok:
                self._showInfo(InfoBarIcon.ERROR, "XR 串口桥启动失败", message)
                return
            self._runWebTask("Web 服务启动", self.webServiceManager.start_service)

        def _stopWebRelay(self) -> None:
            self.webServiceManager.stop_service()
            self.xrBridgeManager.stop()
            self._showInfo(InfoBarIcon.INFORMATION, "服务已停止", "Quest 页面服务和本地 XR 串口桥都已关闭")

        def _connectSerial(self) -> None:
            if serial is None:
                self._showInfo(InfoBarIcon.ERROR, "缺少依赖", "请先安装 pyserial")
                return

            port = self.overviewPage.selectedPort()
            if not port:
                self._showInfo(InfoBarIcon.WARNING, "未选择串口", "请先选择一个可用串口")
                return

            baud = self.overviewPage.selectedBaud()
            self._disconnectSerial(silent=True)
            self.worker = SerialWorker(port, baud)
            self.worker.start()
            self.last_error = ""
            self.lastOutputInterface = None
            self.lastStatusSignature = None
            self._updateConnectionStatusItem()
            self._showInfo(InfoBarIcon.INFORMATION, "正在连接", f"{port} @ {baud}")

        def _disconnectSerial(self, checked: bool = False, silent: bool = False) -> None:
            _ = checked
            if self.worker is not None:
                self.worker.stop_worker()
                self.worker = None
            self.last_connected = False
            self.last_rate_hz = 0.0
            self.lastOutputInterface = None
            self._updateConnectionStatusItem()
            if not silent:
                self._showInfo(InfoBarIcon.INFORMATION, "已断开", "串口连接已关闭")

        def _maybeNotifyStatus(self, connected: bool, error: str) -> None:
            signature = link_status_summary(connected, error)
            if signature == self.lastStatusSignature:
                return
            self.lastStatusSignature = signature

            level, text, hint = signature
            if level == "success":
                self._showInfo(InfoBarIcon.SUCCESS, text, hint)
            elif level == "error":
                self._showInfo(InfoBarIcon.ERROR, text, hint)

        def _refresh(self) -> None:
            if self.worker is not None:
                latest, stats, rate_hz, connected, error, xr_status, xr_lines = self.worker.snapshot()
                if latest is not None:
                    self.last_frame = latest
                    self.lastOutputInterface = latest.output_if
                self.last_stats = stats
                self.last_rate_hz = rate_hz
                self.last_connected = connected
                self.last_error = error
                self.last_xr_status = xr_status
                self.last_xr_lines = xr_lines
                if not connected:
                    self.lastOutputInterface = None
                self._updateConnectionStatusItem()
                self._maybeNotifyStatus(connected, error)

            self.overviewPage.updateView(
                self.last_frame,
                self.last_stats,
                self.last_rate_hz,
                self.last_connected,
                self.last_error,
            )
            relay_health = self.webServiceManager.fetch_health()
            bridge_health = self.xrBridgeManager.snapshot()
            self.last_xr_bridge = bridge_health
            device_status = bridge_health.xr_status if bridge_health.xr_status.raw_line else self.last_xr_status
            self.last_xr_status = device_status
            self.last_xr_lines = bridge_health.xr_lines
            self.xrControlPage.applyDeviceStatus(device_status)
            self.xrControlPage.applyRelayHealth(
                relay_health,
                bridge_health,
                [*self.webServiceManager.recent_logs(), *self.last_xr_lines],
            )
            self.xrControlPage.applyQuestStatus(relay_health)
            self.previewPage.updateView(self.last_frame)
            self.monitorPage.updateView(
                self.last_frame,
                self.last_stats,
                self.last_rate_hz,
                self.last_connected,
                self.last_error,
            )

        def closeEvent(self, event) -> None:  # type: ignore[override]
            self._disconnectSerial(silent=True)
            self.webServiceManager.stop_service()
            self.xrBridgeManager.stop()
            super().closeEvent(event)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CCtrl serial monitor rebuilt with PyQt-Fluent-Widgets")
    parser.add_argument("--port", default="COM6", help="默认串口，例如 COM6")
    parser.add_argument("--baud", type=int, default=115200, choices=(115200, 2000000), help="默认波特率")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    if QT_IMPORT_ERROR is not None:
        sys.stderr.write("缺少 PyQt5。\n请先安装：pip install PyQt5\n")
        return 1

    if QFLUENT_IMPORT_ERROR is not None:
        sys.stderr.write(
            "缺少 qfluentwidgets。\n"
            "官方库：https://github.com/zhiyiYo/PyQt-Fluent-Widgets\n"
            "安装命令：pip install PyQt-Fluent-Widgets -i https://pypi.org/simple/\n"
        )
        return 1

    if hasattr(QApplication, "setHighDpiScaleFactorRoundingPolicy"):
        QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    if hasattr(Qt, "AA_EnableHighDpiScaling"):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    if hasattr(Qt, "AA_UseHighDpiPixmaps"):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    if hasattr(Qt, "AA_DontCreateNativeWidgetSiblings"):
        QApplication.setAttribute(Qt.AA_DontCreateNativeWidgetSiblings)

    app = QApplication(sys.argv if argv is None else [sys.argv[0], *argv])
    window = PreviewMonitorWindow(args.port, args.baud)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
