from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError as exc:  # pragma: no cover - runtime guard
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = exc
else:  # pragma: no cover - runtime guard
    SERIAL_IMPORT_ERROR = None

try:
    from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
    from PyQt5.QtGui import QColor, QFont, QLinearGradient, QPainter, QPainterPath, QPen
    from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QWidget
except ImportError as exc:  # pragma: no cover - runtime guard
    PYQT_IMPORT_ERROR = exc
else:  # pragma: no cover - runtime guard
    PYQT_IMPORT_ERROR = None


RM_SOF = 0xA5
RM_CMD_ID = 0x0302
RM_DATA_LEN = 30
FRAME_LEN = 5 + 2 + RM_DATA_LEN + 2
KEY5_MASK = 0x10

# Match the transmitted/controller pose contract directly in the viewer:
# the viewer world frame is left-handed with +Z up, while world-space Y is
# flipped from the raw stream. Local/end-effector axes keep the streamed pose
# basis and are not mirrored through this helper.


def map_world_point_to_view(point: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (point[0], -point[1], point[2])


def half_to_float(half_word: int) -> float:
    sign = -1.0 if (half_word & 0x8000) else 1.0
    exponent = (half_word >> 10) & 0x1F
    mantissa = half_word & 0x03FF

    if exponent == 0:
        if mantissa == 0:
            return sign * 0.0
        return sign * (mantissa / 1024.0) * (2.0 ** -14)

    if exponent == 0x1F:
        if mantissa == 0:
            return sign * math.inf
        return math.nan

    return sign * (1.0 + mantissa / 1024.0) * (2.0 ** (exponent - 15))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def vec_add(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vec_norm(a: Tuple[float, float, float]) -> float:
    return math.sqrt(vec_dot(a, a))


def vec_normalize(a: Tuple[float, float, float]) -> Tuple[float, float, float]:
    length = vec_norm(a)
    if length <= 1e-9:
        return (0.0, 0.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)


def quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    length = math.sqrt(sum(component * component for component in q))
    if length <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(component / length for component in q)  # type: ignore[return-value]


def quat_mul(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    return (q[0], -q[1], -q[2], -q[3])


def quat_inverse(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm_sq = sum(component * component for component in q)
    if norm_sq <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    conj = quat_conjugate(q)
    return tuple(component / norm_sq for component in conj)  # type: ignore[return-value]


def quat_from_euler_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return quat_normalize(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )


def quat_to_matrix(q: Tuple[float, float, float, float]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    w, x, y, z = quat_normalize(q)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def rotate_point(point: Tuple[float, float, float], quat: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    matrix = quat_to_matrix(quat)
    x, y, z = point
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


def format_vec3(value: Tuple[float, float, float]) -> str:
    return f"({value[0]:+.1f}, {value[1]:+.1f}, {value[2]:+.1f})"


def format_quat(value: Tuple[float, float, float, float]) -> str:
    return f"({value[0]:+.3f}, {value[1]:+.3f}, {value[2]:+.3f}, {value[3]:+.3f})"


def decode_status_name(status_err: int) -> str:
    status = (status_err >> 4) & 0x0F
    return {
        0: "IDLE",
        1: "ACTIVE",
        2: "DISCONNECT",
    }.get(status, f"S{status}")


def decode_error_flags(error_flags: int) -> str:
    labels = []
    if error_flags & 0x01:
        labels.append("CRC")
    if error_flags & 0x02:
        labels.append("TIMEOUT")
    if error_flags & 0x04:
        labels.append("PARSE")
    if error_flags & 0x08:
        labels.append("TOPO")
    return ",".join(labels) if labels else "OK"


@dataclass
class RawFrame:
    seq: int
    status_err: int
    key_flags: int
    delta_key: int
    pos: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]
    raw_attitude: Tuple[float, float, float, float]
    pose_mode: str
    attitude_format: str
    mode_flags: int
    timestamp: float


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[RawFrame]:
        frames: List[RawFrame] = []
        if not chunk:
            return frames

        self._buffer.extend(chunk)

        while True:
            start = self._buffer.find(bytes([RM_SOF]))
            if start < 0:
                self._buffer.clear()
                break

            if start > 0:
                del self._buffer[:start]

            if len(self._buffer) < FRAME_LEN:
                break

            frame = bytes(self._buffer[:FRAME_LEN])
            data_len = frame[1] | (frame[2] << 8)
            cmd_id = frame[5] | (frame[6] << 8)

            if data_len != RM_DATA_LEN or cmd_id != RM_CMD_ID:
                del self._buffer[0]
                continue

            payload = frame[7:-2]
            try:
                frames.append(self._decode_frame(frame[3], payload))
            except Exception:
                del self._buffer[0]
                continue

            del self._buffer[:FRAME_LEN]

        return frames

    @staticmethod
    def _decode_frame(seq: int, payload: bytes) -> RawFrame:
        if len(payload) != RM_DATA_LEN:
            raise ValueError("invalid payload length")

        status_err = payload[0]
        key_flags = int.from_bytes(payload[1:3], "little", signed=False)
        delta_key = payload[3]
        pos_words = struct.unpack_from("<3H", payload, 8)
        att_words = struct.unpack_from("<4H", payload, 14)
        mode_flags = payload[22]

        pos = tuple(half_to_float(word) for word in pos_words)  # type: ignore[assignment]

        attitude_format = "QUAT" if (mode_flags & 0x04) else "EUL"
        pose_mode = "ABS" if (mode_flags & 0x02) else "REL"

        if attitude_format == "QUAT":
            raw_attitude = tuple(half_to_float(word) for word in att_words)  # type: ignore[assignment]
            quat = quat_normalize(raw_attitude)
        else:
            roll = half_to_float(att_words[0])
            pitch = half_to_float(att_words[1])
            yaw = half_to_float(att_words[2])
            raw_attitude = (roll, pitch, yaw, 0.0)
            quat = quat_from_euler_deg(roll, pitch, yaw)

        return RawFrame(
            seq=seq,
            status_err=status_err,
            key_flags=key_flags,
            delta_key=delta_key,
            pos=pos,
            quat=quat,
            raw_attitude=raw_attitude,
            pose_mode=pose_mode,
            attitude_format=attitude_format,
            mode_flags=mode_flags,
            timestamp=time.time(),
        )


class ClutchedPose:
    def __init__(self) -> None:
        self.display_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.display_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        self._initialized = False
        self._key_active_prev = False
        self._anchor_raw_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._anchor_raw_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        self._anchor_display_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._anchor_display_quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    def update(self, frame: RawFrame) -> bool:
        key_active = bool(frame.delta_key or (frame.key_flags & KEY5_MASK))

        if not self._initialized:
            self.display_pos = map_world_point_to_view(frame.pos)
            self.display_quat = frame.quat
            self._initialized = True

        if key_active and not self._key_active_prev:
            self._anchor_raw_pos = frame.pos
            self._anchor_raw_quat = frame.quat
            self._anchor_display_pos = self.display_pos
            self._anchor_display_quat = self.display_quat

        if key_active:
            delta_pos = map_world_point_to_view(vec_sub(frame.pos, self._anchor_raw_pos))
            delta_quat = quat_mul(frame.quat, quat_inverse(self._anchor_raw_quat))
            self.display_pos = vec_add(self._anchor_display_pos, delta_pos)
            self.display_quat = quat_normalize(quat_mul(delta_quat, self._anchor_display_quat))

        self._key_active_prev = key_active
        return key_active


class SerialReader(threading.Thread):
    def __init__(self, port: str, baudrate: int) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._parser = FrameParser()
        self._latest_frame: Optional[RawFrame] = None
        self._frame_count = 0
        self._error: str = ""
        self._serial = None

    def run(self) -> None:
        if serial is None:
            with self._lock:
                self._error = "pyserial is not installed"
            return

        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.05)
        except Exception as exc:
            with self._lock:
                self._error = f"open failed: {exc}"
            return

        try:
            while not self._stop_event.is_set():
                assert self._serial is not None
                waiting = int(getattr(self._serial, "in_waiting", 0) or 1)
                chunk = self._serial.read(waiting)
                if not chunk:
                    time.sleep(0.005)
                    continue

                frames = self._parser.feed(chunk)
                if not frames:
                    continue

                with self._lock:
                    for frame in frames:
                        self._latest_frame = frame
                        self._frame_count += 1
        except Exception as exc:
            with self._lock:
                self._error = f"read failed: {exc}"
        finally:
            try:
                if self._serial is not None:
                    self._serial.close()
            except Exception:
                pass

    def snapshot(self) -> Tuple[Optional[RawFrame], int, str]:
        with self._lock:
            return self._latest_frame, self._frame_count, self._error

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=1.0)


class CubeViewerWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(960, 700)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self._dragging = False
        self._last_mouse_pos = None
        self._camera_yaw = 35.0
        self._camera_pitch = 25.0
        self._camera_distance = 620.0
        self._focal_length = 440.0
        self._cube_size = 72.0

        self._display_pose = ClutchedPose()
        self._latest_frame: Optional[RawFrame] = None
        self._frame_count = 0
        self._key_active = False

        self._mono_font = QFont("Consolas", 10)
        self._mono_font.setStyleHint(QFont.Monospace)

        self._palette = {
            "bg_top": QColor("#111821"),
            "bg_bottom": QColor("#0A0F15"),
            "grid_minor": QColor(85, 100, 120, 55),
            "grid_major": QColor(120, 145, 170, 95),
            "axis_x": QColor("#FF7B72"),
            "axis_y": QColor("#3DD9B4"),
            "axis_z": QColor("#4DA3FF"),
            "cube": QColor("#C8D4E0"),
            "cube_border": QColor("#92B4D0"),
            "local_x": QColor("#FF8A80"),
            "local_y": QColor("#7CFFCB"),
            "local_z": QColor("#87B7FF"),
            "overlay": QColor("#E8F1FA"),
            "warn": QColor("#FFD166"),
        }

    def set_frame(self, frame: Optional[RawFrame], frame_count: int, key_active: bool) -> None:
        if frame is not None:
            self._latest_frame = frame
            self._frame_count = frame_count
            self._key_active = key_active
            self._display_pose.update(frame)
        self.update()

    def _camera_basis(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        yaw = math.radians(self._camera_yaw)
        pitch = math.radians(clamp(self._camera_pitch, -85.0, 85.0))

        camera_position = (
            self._camera_distance * math.cos(pitch) * math.cos(yaw),
            self._camera_distance * math.cos(pitch) * math.sin(yaw),
            self._camera_distance * math.sin(pitch),
        )
        target = (0.0, 0.0, 0.0)

        forward = vec_normalize(vec_sub(target, camera_position))
        world_up = (0.0, 0.0, 1.0)
        right = vec_normalize(vec_cross(forward, world_up))
        if vec_norm(right) <= 1e-9:
            right = (1.0, 0.0, 0.0)
        up = vec_cross(right, forward)
        return camera_position, right, up, forward

    def _project(self, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        camera_position, right, up, forward = self._camera_basis()
        rel = vec_sub(point, camera_position)
        x_cam = vec_dot(rel, right)
        y_cam = vec_dot(rel, up)
        z_cam = vec_dot(rel, forward)
        depth = max(1.0, z_cam)

        width = max(1.0, float(self.width()))
        height = max(1.0, float(self.height()))
        center_x = width * 0.5
        center_y = height * 0.54
        scale = self._focal_length / depth
        return (center_x + x_cam * scale, center_y - y_cam * scale, depth)

    def _cube_center(self) -> Tuple[float, float, float]:
        pos = self._display_pose.display_pos
        return (-pos[0], -pos[1], pos[2])

    def _display_space_point(self, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return (point[0], -point[1], point[2])

    def _local_display_point_to_world(
        self,
        point: Tuple[float, float, float],
        quat: Tuple[float, float, float, float],
        pos: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        return vec_add(rotate_point(self._display_space_point(point), quat), pos)

    def _build_cube_vertices(self) -> List[Tuple[float, float, float]]:
        half = self._cube_size * 0.5
        local = [
            (-half, -half, -half),
            (half, -half, -half),
            (half, half, -half),
            (-half, half, -half),
            (-half, -half, half),
            (half, -half, half),
            (half, half, half),
            (-half, half, half),
        ]
        quat = self._display_pose.display_quat
        pos = self._cube_center()
        return [self._local_display_point_to_world(vertex, quat, pos) for vertex in local]

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._last_mouse_pos = event.pos()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
        if self._dragging and self._last_mouse_pos is not None:
            delta = event.pos() - self._last_mouse_pos
            self._last_mouse_pos = event.pos()
            self._camera_yaw -= delta.x() * 0.45
            self._camera_pitch += delta.y() * 0.35
            self._camera_pitch = clamp(self._camera_pitch, -80.0, 80.0)
            self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            self._dragging = False
            self._last_mouse_pos = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        delta = event.angleDelta().y()
        if delta:
            factor = 0.88 if delta < 0 else 1.12
            self._camera_distance = clamp(self._camera_distance * factor, 220.0, 2000.0)
            self.update()
            event.accept()
            return
        super().wheelEvent(event)

    def paintEvent(self, event) -> None:  # type: ignore[override]
        _ = event
        painter = QPainter(self)
        painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)

        rect = QRectF(self.rect().adjusted(6, 6, -6, -6))
        path = QPainterPath()
        path.addRoundedRect(rect, 18.0, 18.0)
        painter.setClipPath(path)

        background = QLinearGradient(rect.topLeft(), rect.bottomLeft())
        background.setColorAt(0.0, self._palette["bg_top"])
        background.setColorAt(1.0, self._palette["bg_bottom"])
        painter.fillRect(rect, background)

        painter.setPen(QPen(QColor(80, 100, 120, 90), 1.0))
        painter.drawRoundedRect(rect, 18.0, 18.0)

        self._draw_grid(painter)
        self._draw_axes(painter)
        self._draw_cube(painter)
        self._draw_overlay(painter)

    def _draw_grid(self, painter: QPainter) -> None:
        grid_range = 220
        step = 20
        for value in range(-grid_range, grid_range + 1, step):
            color = self._palette["grid_major"] if value == 0 else self._palette["grid_minor"]
            painter.setPen(QPen(color, 1.0))

            p1 = self._project((value, -grid_range, 0.0))
            p2 = self._project((value, grid_range, 0.0))
            painter.drawLine(QPointF(p1[0], p1[1]), QPointF(p2[0], p2[1]))

            p3 = self._project((-grid_range, value, 0.0))
            p4 = self._project((grid_range, value, 0.0))
            painter.drawLine(QPointF(p3[0], p3[1]), QPointF(p4[0], p4[1]))

    def _draw_axes(self, painter: QPainter) -> None:
        origin = self._project((0.0, 0.0, 0.0))
        x_tip = self._project((120.0, 0.0, 0.0))
        y_tip = self._project((0.0, -120.0, 0.0))
        z_tip = self._project((0.0, 0.0, 120.0))

        painter.setPen(QPen(self._palette["axis_x"], 2.2))
        painter.drawLine(QPointF(origin[0], origin[1]), QPointF(x_tip[0], x_tip[1]))
        painter.setPen(QPen(self._palette["axis_y"], 2.2))
        painter.drawLine(QPointF(origin[0], origin[1]), QPointF(y_tip[0], y_tip[1]))
        painter.setPen(QPen(self._palette["axis_z"], 2.2))
        painter.drawLine(QPointF(origin[0], origin[1]), QPointF(z_tip[0], z_tip[1]))

    def _draw_cube(self, painter: QPainter) -> None:
        vertices = self._build_cube_vertices()
        projected = [self._project(vertex) for vertex in vertices]

        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        painter.setPen(QPen(self._palette["cube_border"], 1.6))
        painter.setBrush(self._palette["cube"])
        for a, b in edges:
            painter.drawLine(QPointF(projected[a][0], projected[a][1]), QPointF(projected[b][0], projected[b][1]))

        center = self._cube_center()
        quat = self._display_pose.display_quat
        axis_targets = (
            self._local_display_point_to_world((60.0, 0.0, 0.0), quat, center),
            self._local_display_point_to_world((0.0, 60.0, 0.0), quat, center),
            self._local_display_point_to_world((0.0, 0.0, 60.0), quat, center),
        )
        center_proj = self._project(center)

        painter.setPen(QPen(self._palette["local_x"], 1.8))
        end = self._project(axis_targets[0])
        painter.drawLine(QPointF(center_proj[0], center_proj[1]), QPointF(end[0], end[1]))
        painter.setPen(QPen(self._palette["local_y"], 1.8))
        end = self._project(axis_targets[1])
        painter.drawLine(QPointF(center_proj[0], center_proj[1]), QPointF(end[0], end[1]))
        painter.setPen(QPen(self._palette["local_z"], 1.8))
        end = self._project(axis_targets[2])
        painter.drawLine(QPointF(center_proj[0], center_proj[1]), QPointF(end[0], end[1]))

    def _draw_overlay(self, painter: QPainter) -> None:
        frame = self._latest_frame
        painter.setFont(self._mono_font)
        painter.setPen(self._palette["overlay"])

        lines = ["RS232 3D Viewer  |  Drag to orbit  |  Wheel to zoom  |  KEY5 acts as clutch"]
        lines.append("World=Mapped Pose (left-handed, Z-up, Y flipped from raw)  |  Local axes follow the streamed pose quaternion")

        if frame is None:
            lines.append("Waiting for a valid RS232 frame...")
        else:
            lines.extend(
                [
                    f"SEQ={frame.seq}   KEY5={'ON' if self._key_active else 'OFF'}   Frames={self._frame_count}",
                    f"Status={decode_status_name(frame.status_err)}   Err={decode_error_flags(frame.status_err & 0x0F)}",
                    f"Mode={frame.pose_mode}/{frame.attitude_format}   modeFlags=0x{frame.mode_flags:02X}",
                    f"Raw XYZ(mm)={format_vec3(frame.pos)}",
                    f"View XYZ(mm)={format_vec3(self._display_pose.display_pos)}",
                    f"Raw Att={format_quat(frame.raw_attitude)}",
                    f"View Quat={format_quat(self._display_pose.display_quat)}",
                ]
            )

        padding_x = 18
        padding_y = 18
        line_height = 22
        text_y = padding_y + 12
        for index, text in enumerate(lines):
            if index == 0:
                painter.setFont(QFont("Consolas", 11, QFont.Bold))
                painter.setPen(self._palette["warn"])
            else:
                painter.setFont(self._mono_font)
                painter.setPen(self._palette["overlay"])
            painter.drawText(QPointF(padding_x, text_y + index * line_height), text)


class MainWindow(QMainWindow):
    def __init__(self, port: str, baudrate: int) -> None:
        super().__init__()
        self.setWindowTitle("RS232 3D Viewer")
        self.resize(1180, 780)

        self._reader = SerialReader(port, baudrate)
        self._viewer = CubeViewerWidget(self)
        self.setCentralWidget(self._viewer)

        self._status = QLabel(self)
        self._status.setText("Starting...")
        self.statusBar().addPermanentWidget(self._status)

        self._last_frame_count = 0
        self._timer = QTimer(self)
        self._timer.setInterval(20)
        self._timer.timeout.connect(self._poll_serial)

        self._reader.start()
        self._timer.start()

    def _poll_serial(self) -> None:
        frame, frame_count, error = self._reader.snapshot()
        if error:
            self._status.setText(error)
            return

        if frame is not None and frame_count != self._last_frame_count:
            self._last_frame_count = frame_count
            key_active = bool(frame.delta_key or (frame.key_flags & KEY5_MASK))
            self._viewer.set_frame(frame, frame_count, key_active)
            self._status.setText(
                f"Port={self._reader.port}  Baud={self._reader.baudrate}  Frames={frame_count}  "
                f"SEQ={frame.seq}  KEY5={'ON' if key_active else 'OFF'}  Pose={frame.pose_mode}  Att={frame.attitude_format}"
            )

    def closeEvent(self, event) -> None:  # type: ignore[override]
        try:
            self._timer.stop()
            self._reader.stop()
        finally:
            super().closeEvent(event)


def resolve_port(requested_port: Optional[str]) -> str:
    if requested_port:
        return requested_port

    if list_ports is None:
        raise SystemExit("pyserial is required to enumerate ports")

    ports = [port.device for port in list_ports.comports()]
    if len(ports) == 1:
        return ports[0]

    if not ports:
        raise SystemExit("No serial ports found. Pass --port COMx manually.")

    print("Available serial ports:")
    for port in ports:
        print(f"  {port}")
    raise SystemExit("Multiple ports found. Pass --port to choose one.")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simple RS232 3D pose viewer")
    parser.add_argument("--port", help="Serial port, for example COM6")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate, default: 115200")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    if SERIAL_IMPORT_ERROR is not None:
        print("pyserial is required. Install it before running this viewer.", file=sys.stderr)
        print(f"Import error: {SERIAL_IMPORT_ERROR}", file=sys.stderr)
        return 1

    if PYQT_IMPORT_ERROR is not None:
        print("PyQt5 is required. Install it before running this viewer.", file=sys.stderr)
        print(f"Import error: {PYQT_IMPORT_ERROR}", file=sys.stderr)
        return 1

    args = parse_args(argv)
    port = resolve_port(args.port)

    app = QApplication(sys.argv if argv is None else [sys.argv[0], *argv])
    app.setApplicationName("RS232 3D Viewer")

    window = MainWindow(port, args.baud)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
