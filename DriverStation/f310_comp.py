#!/usr/bin/env python3 

"""Forward F310 GUI UDP packets to the Arduino with minimal processing."""

from __future__ import annotations

import json
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import List, Optional, Tuple

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from system_operations.arduino_serial_commuication.arduino_serial_commuication import (
    serialCommands,
)
from DriverStation.autonomous.excavation import get_sequence as get_excavation_sequence
from DriverStation.autonomous.dump import get_sequence as get_dump_sequence
from DriverStation.autonomous.transverse import get_sequence as get_transverse_sequence

try:
    import cv2  # type: ignore
except Exception:
    cv2 = None  # type: ignore[assignment]

SYNC = 0xA6
LISTEN_ADDR: Tuple[str, int] = ("0.0.0.0", 11000)
SEND_RATE_HZ = 40.0
DISCOVERY_PORT = 11010
DISCOVERY_MAGIC = b"F310_DISCOVERY_V1"
TELEMETRY_PORT = 10000
TELEMETRY_RATE_HZ = 2.0
CAMERA_HTTP_PORT = 8081
CAMERA_DEVICE_INDEX = -1
CAMERA_WIDTH = 960
CAMERA_HEIGHT = 540
CAMERA_FPS = 24.0
CAMERA_JPEG_QUALITY = 70
CAMERA_LOW_LATENCY = True
CAMERA_DEVICE_SCAN_LIMIT = 8
MAX_DRIVE_OUTPUT = 127         #127 is max speed for motor controller do not change
ACCEL_LIMIT_PER_SEC = 50.0
DECEL_LIMIT_PER_SEC = 60.0
DECEL_NEAR_ZERO_THRESHOLD = 5.0
DECEL_NEAR_ZERO_PER_SEC = 90.0
ARM_MISS_TOLERANCE = 3
HEADSET_LISTEN_ADDR: Tuple[str, int] = ("0.0.0.0", 12002)  # matches Unity default; adjust only if Unity IP/port changes
HEADSET_STALE_SEC = 0.5
HEADSET_YAW_RANGE_DEG = (-90.0, 90.0)
SERVO_RANGE = (0, 180)
SERVO_CENTER = 90
SERVO_RESEND_INTERVAL = 0.5
DPAD_X_AXIS_INDEX = 6
DPAD_Y_AXIS_INDEX = 7
DPAD_UP_BUTTON_INDEX = 13
DPAD_ACTIVE_THRESHOLD = 32
DPAD_SERVO_SPEED_DEG_PER_SEC = 120.0

AUTO_PROGRAMS = {
    "excavation": get_excavation_sequence,
    "dump": get_dump_sequence,
    "transverse": get_transverse_sequence,
}

AUTO_BUTTONS = {
    0: "excavation",
    1: "dump",
    2: "transverse",
}


@dataclass
class VideoSettings:
    width: int
    height: int
    fps: float
    quality: int
    device_index: int
    low_latency: bool


def clamp_i8(value: int) -> int:
    return max(-127, min(127, value))


def clamp_drive(value: int) -> int:
    return max(-MAX_DRIVE_OUTPUT, min(MAX_DRIVE_OUTPUT, value))


def _scale_to_drive(value: int) -> int:
    if value == 0:
        return 0
    scaled = int(round(value / 127.0 * MAX_DRIVE_OUTPUT))
    return clamp_drive(scaled)


class AccelLimiter:
    """Acceleration curve with independent accel/decel behavior."""

    def __init__(
        self,
        accel_rate: float,
        decel_rate: float,
        decel_near_zero: float,
        near_zero_threshold: float,
    ) -> None:
        self.accel_rate = max(1.0, float(accel_rate))
        self.decel_rate = max(1.0, float(decel_rate))
        self.decel_near_zero = max(1.0, float(decel_near_zero))
        self.near_zero_threshold = max(0.0, float(near_zero_threshold))
        self.left = 0.0
        self.right = 0.0

    def reset(self, left: float = 0.0, right: float = 0.0) -> None:
        self.left = float(left)
        self.right = float(right)

    def update(self, target_left: float, target_right: float, dt: float) -> Tuple[float, float]:
        self.left = self._apply(self.left, target_left, dt)
        self.right = self._apply(self.right, target_right, dt)
        return self.left, self.right

    def _apply(self, current: float, target: float, dt: float) -> float:
        if target == current:
            return target
        increasing = abs(target) > abs(current)
        if increasing:
            rate = self.accel_rate
        else:
            rate = self.decel_near_zero if abs(target) <= self.near_zero_threshold else self.decel_rate
        max_delta = rate * max(dt, 0.0)
        if target > current:
            return min(current + max_delta, target)
        return max(current - max_delta, target)


def decode_packet(packet: bytes) -> Optional[Tuple[Tuple[int, ...], Tuple[int, ...], bool]]:
    if len(packet) < 14 or packet[0] != SYNC:
        return None
    checksum = 0
    for byte in packet[:-1]:
        checksum ^= byte
    if checksum != packet[-1]:
        return None
    axes = struct.unpack("bbbbbbbb", packet[3:11])
    buttons_mask = packet[11] | (packet[12] << 8)
    buttons = tuple((buttons_mask >> bit) & 0x01 for bit in range(16))
    armed = bool(packet[2] & 0x01)
    return axes, buttons, armed


def _clamp_servo(angle: int) -> int:
    lo, hi = SERVO_RANGE
    return max(lo, min(hi, angle))


def _yaw_to_servo(yaw_deg: float) -> int:
    lo, hi = HEADSET_YAW_RANGE_DEG
    yaw_clamped = max(lo, min(hi, yaw_deg))
    proportion = (yaw_clamped - lo) / (hi - lo) if hi != lo else 0.5
    angle = int(round(SERVO_RANGE[0] + proportion * (SERVO_RANGE[1] - SERVO_RANGE[0])))
    return _clamp_servo(angle)


class DriverStationTracker:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._addr: Optional[str] = None

    def update(self, ip: str) -> None:
        if not ip:
            return
        with self._lock:
            self._addr = ip

    def current(self) -> Optional[str]:
        with self._lock:
            return self._addr


class VideoCaptureWorker:
    def __init__(self, settings: VideoSettings) -> None:
        if cv2 is None:
            raise SystemExit("OpenCV (cv2) is required to capture video; pip install opencv-python")
        self._settings = settings
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[bytes] = None
        self._actual_fps = 0.0
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="VideoCapture", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)

    def latest_frame(self) -> Optional[bytes]:
        with self._frame_lock:
            return None if self._latest_frame is None else bytes(self._latest_frame)

    def actual_fps(self) -> float:
        with self._frame_lock:
            return self._actual_fps

    @property
    def settings(self) -> VideoSettings:
        return self._settings

    @staticmethod
    def _backend_candidates() -> List[int]:
        assert cv2 is not None
        if sys.platform.startswith("win"):
            return [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        return [cv2.CAP_ANY]

    @staticmethod
    def _device_label(index: int, backend: int) -> str:
        backend_name = "default"
        if cv2 is not None:
            if backend == getattr(cv2, "CAP_DSHOW", -9999):
                backend_name = "dshow"
            elif backend == getattr(cv2, "CAP_MSMF", -9999):
                backend_name = "msmf"
            elif backend == getattr(cv2, "CAP_ANY", -9999):
                backend_name = "any"
        return f"index {index} ({backend_name})"

    def _open_capture(self) -> Tuple[Optional[object], Optional[str]]:
        assert cv2 is not None
        preferred_indices = [self._settings.device_index] if self._settings.device_index >= 0 else []
        scanned_indices = [idx for idx in range(CAMERA_DEVICE_SCAN_LIMIT) if idx not in preferred_indices]
        candidates = preferred_indices + scanned_indices

        for index in candidates:
            for backend in self._backend_candidates():
                cap = cv2.VideoCapture(index, backend)
                if not cap.isOpened():
                    cap.release()
                    continue
                ok, frame = cap.read()
                if ok and frame is not None:
                    label = self._device_label(index, backend)
                    self._settings.device_index = index
                    return cap, label
                cap.release()
        return None, None

    def _run(self) -> None:
        assert cv2 is not None
        cap, selected_label = self._open_capture()
        if cap is None:
            print(
                f"[camera] Unable to open any camera device. Tried preferred index "
                f"{self._settings.device_index if self._settings.device_index >= 0 else 'auto'} "
                f"and scanned indices 0-{CAMERA_DEVICE_SCAN_LIMIT - 1}.",
                file=sys.stderr,
            )
            return
        print(f"[camera] Using {selected_label}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._settings.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._settings.height))
        if self._settings.fps > 0:
            cap.set(cv2.CAP_PROP_FPS, float(self._settings.fps))
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        encode_quality = int(max(1, min(100, self._settings.quality)))
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), encode_quality]
        interval = 0.0 if self._settings.low_latency else 1.0 / max(self._settings.fps, 0.1)
        last_frame_time = time.monotonic()
        try:
            while not self._stop_event.is_set():
                ret, frame = cap.read()
                now = time.monotonic()
                if ret:
                    ok, buffer = cv2.imencode(".jpg", frame, encode_params)
                    if ok:
                        with self._frame_lock:
                            self._latest_frame = buffer.tobytes()
                            delta = max(1e-6, now - last_frame_time)
                            instant = 1.0 / delta
                            self._actual_fps = instant if self._actual_fps == 0.0 else (0.8 * self._actual_fps + 0.2 * instant)
                        last_frame_time = now
                else:
                    time.sleep(0.05)
                loop_elapsed = time.monotonic() - now
                if interval > 0.0:
                    remaining = interval - loop_elapsed
                    if remaining > 0:
                        self._stop_event.wait(remaining)
                else:
                    self._stop_event.wait(0.001)
        finally:
            cap.release()


class CameraRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path not in ("/", "/frame"):
            self.send_error(404, "Not Found")
            return
        frame = getattr(self.server, "video_source").latest_frame()  # type: ignore[attr-defined]
        if frame is None:
            self.send_error(503, "Camera frame unavailable")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def log_message(self, fmt: str, *args: object) -> None:
        pass


class CameraHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, address: Tuple[str, int], video_source: VideoCaptureWorker) -> None:
        self.video_source = video_source
        super().__init__(address, CameraRequestHandler)
        self.daemon_threads = True


class TelemetryBroadcaster:
    def __init__(self, tracker: DriverStationTracker, interval: float = 0.5) -> None:
        self._tracker = tracker
        self._interval = max(0.2, interval)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="TelemetryBroadcaster", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        with sock:
            while not self._stop_event.is_set():
                target = self._tracker.current()
                payload = self._build_payload(target)
                if target and payload:
                    try:
                        sock.sendto(payload, (target, TELEMETRY_PORT))
                    except OSError:
                        pass
                self._stop_event.wait(self._interval)

    @staticmethod
    def _guess_advertised_host(driver_station_ip: Optional[str]) -> Optional[str]:
        if not driver_station_ip:
            return None
        probe: Optional[socket.socket] = None
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe.connect((driver_station_ip, 9))
            return probe.getsockname()[0]
        except OSError:
            return None
        finally:
            try:
                if probe is not None:
                    probe.close()
            except Exception:
                pass

    def _build_payload(self, driver_station_ip: Optional[str]) -> Optional[bytes]:
        camera_host = self._guess_advertised_host(driver_station_ip)
        camera_url = None
        if camera_host:
            camera_url = f"http://{camera_host}:{CAMERA_HTTP_PORT}/frame"
        payload = {
            "camera_url": camera_url,
            "source": "f310_comp",
            "telemetry_port": TELEMETRY_PORT,
        }
        return json.dumps(payload).encode("utf-8")


def auto_local_ip() -> str:
    probe: Optional[socket.socket] = None
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        host = probe.getsockname()[0]
    except OSError:
        try:
            host = socket.gethostbyname(socket.gethostname())
        except OSError:
            host = "127.0.0.1"
    finally:
        try:
            if probe is not None:
                probe.close()
        except Exception:
            pass
    return host


def _open_link(shared: dict, stop_event: threading.Event) -> Optional[serialCommands]:
    while not stop_event.is_set():
        try:
            link = serialCommands()
        except Exception as exc:
            with shared["lock"]:
                shared["serial_status"] = f"Arduino not found: {exc}"
            stop_event.wait(1.0)
            continue
        with shared["lock"]:
            shared["serial_status"] = "connected"
        return link
    return None


def _receiver(
    sock: socket.socket,
    tracker: DriverStationTracker,
    shared: dict,
    stop_event: threading.Event,
) -> None:
    sock.settimeout(0.2)
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(64)
        except socket.timeout:
            continue
        except OSError:
            break
        tracker.update(addr[0])
        decoded = decode_packet(data)
        if decoded is None:
            continue
        axes, buttons, armed = decoded
        now = time.monotonic()
        cancel_auto = False
        requested_program: Optional[str] = None
        with shared["lock"]:
            prev = shared["last_udp"]
            prev_buttons = shared.get("buttons", tuple())
            shared["axes"] = axes
            shared["buttons"] = buttons
            shared["armed"] = armed
            if armed:
                shared["armed_miss_count"] = 0
                shared["armed_effective"] = True
            else:
                miss = int(shared.get("armed_miss_count", 0)) + 1
                shared["armed_miss_count"] = miss
                if miss > ARM_MISS_TOLERANCE:
                    shared["armed_effective"] = False
            shared["last_udp"] = now
            shared["net_dt"] = 0.0 if prev == 0.0 else now - prev
            if buttons:
                for idx, program in AUTO_BUTTONS.items():
                    prev_val = prev_buttons[idx] if len(prev_buttons) > idx else 0
                    curr_val = buttons[idx] if len(buttons) > idx else 0
                    if curr_val and not prev_val and not shared.get("auto_active", False):
                        requested_program = program
                        break
                if len(buttons) > 4 and buttons[4] and shared.get("auto_active", False):
                    cancel_auto = True

        if requested_program:
            with shared["lock"]:
                shared["auto_requested"] = requested_program

        if cancel_auto:
            _stop_auto(shared, "autonomous cancelled (armed)")


def _parse_headset_payload(data: bytes) -> Tuple[Optional[float], Optional[int]]:
    """Accepts JSON dicts with yaw/yaw_deg/heading or a bare number string/int."""
    text = data.decode(errors="ignore").strip()
    if not text:
        return None, None
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        parsed = None
    if isinstance(parsed, dict):
        if "servo" in parsed:
            try:
                return None, int(parsed["servo"])
            except (TypeError, ValueError):
                pass
        for key in ("yaw", "yaw_deg", "heading"):
            if key in parsed:
                try:
                    return float(parsed[key]), None
                except (TypeError, ValueError):
                    break
    elif isinstance(parsed, (int, float)):
        return None, int(parsed)
    try:
        # If it's a number string, treat it as a servo command (Unity currently sends a mapped angle)
        return None, int(float(text))
    except ValueError:
        return None, None


def _headset_listener(shared: dict, stop_event: threading.Event) -> None:
    """Listen for yaw updates from the headset and convert to servo angles."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(HEADSET_LISTEN_ADDR)
        sock.settimeout(0.3)
    except OSError as exc:
        with shared["lock"]:
            shared["vr_status"] = f"vr disabled: {exc}"
        return

    with sock:
        while not stop_event.is_set():
            try:
                data, _ = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break

            yaw, servo_direct = _parse_headset_payload(data)
            if yaw is None and servo_direct is None:
                continue
            angle = _clamp_servo(servo_direct) if servo_direct is not None else _yaw_to_servo(yaw)
            now = time.monotonic()
            with shared["lock"]:
                shared["vr_yaw_deg"] = yaw
                shared["servo_angle"] = angle
                shared["vr_last_update"] = now
                shared["vr_status"] = "receiving"

def _discovery_responder(tracker: DriverStationTracker, stop_event: threading.Event) -> None:
    """Respond to UDP broadcast discovery packets so the GUI can learn our IP."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", DISCOVERY_PORT))
        sock.settimeout(0.5)
    except OSError as exc:
        print(f"[discovery] disabled: {exc}")
        return

    reply = json.dumps(
        {
            "role": "f310_comp",
            "udp_port": LISTEN_ADDR[1],
            "command_port": 10001,
            "telemetry_port": TELEMETRY_PORT,
        }
    ).encode("utf-8")

    with sock:
        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(128)
            except socket.timeout:
                continue
            except OSError:
                break
            if data.strip() != DISCOVERY_MAGIC:
                continue
            tracker.update(addr[0])
            try:
                sock.sendto(reply, addr)
            except OSError:
                continue

def _maybe_start_auto(shared: dict, now: float) -> None:
    with shared["lock"]:
        requested = shared.get("auto_requested")
        if not requested:
            return
        if shared.get("auto_active"):
            return
        sequence_loader = AUTO_PROGRAMS.get(str(requested))
        try:
            sequence = list(sequence_loader()) if sequence_loader else []
        except Exception as exc:
            shared["serial_status"] = f"auto load failed: {exc}"
            shared["auto_requested"] = None
            return
        if not sequence:
            shared["serial_status"] = f"auto unavailable: {requested}" if requested else "auto unavailable"
            shared["auto_requested"] = None
            return
        shared["auto_requested"] = None
        shared["auto_sequence"] = sequence
        shared["auto_total_steps"] = len(sequence)
        shared["auto_step"] = 0
        shared["auto_step_start"] = 0.0
        shared["auto_step_end"] = 0.0
        shared["auto_step_initialized"] = False
        shared["auto_active"] = True
        shared["auto_m_values"] = [0, 0]
        shared["auto_a_values"] = [0, 0, 0, 0, 0, 0]
        shared["auto_command"] = ""
        shared["auto_values"] = []
        shared["auto_time_remaining"] = 0.0
        shared["auto_program"] = str(requested)
        shared["control_mode"] = "auto"
        shared["serial_status"] = f"running {requested}"


def _stop_auto(shared: dict, reason: str) -> None:
    with shared["lock"]:
        shared["auto_active"] = False
        shared["auto_requested"] = None
        shared["control_mode"] = "manual"
        shared["serial_status"] = reason
        shared["auto_time_remaining"] = 0.0
        shared["auto_command"] = ""
        shared["auto_values"] = []
        shared["auto_m_values"] = [0, 0]
        shared["auto_a_values"] = [0, 0, 0, 0, 0, 0]
        shared["auto_step_initialized"] = False
        shared["auto_step_start"] = 0.0
        shared["auto_step_end"] = 0.0
        shared["auto_step"] = shared.get("auto_total_steps", 0)
        shared["auto_program"] = ""


def _update_auto_state(shared: dict, now: float) -> Optional[Tuple[List[int], List[int]]]:
    while True:
        stop_reason: Optional[str] = None
        continue_loop = False
        result: Optional[Tuple[List[int], List[int]]] = None

        with shared["lock"]:
            if not shared.get("auto_active"):
                return None

            sequence = shared.get("auto_sequence", [])
            idx = shared.get("auto_step", 0)

            if idx >= len(sequence):
                stop_reason = "autonomous complete"
            else:
                entry = sequence[idx]
                initialized = shared.get("auto_step_initialized", False)

                if not initialized:
                    duration = float(entry.get("duration", 0.0) or 0.0)
                    command = entry.get("command")
                    values = entry.get("values", [])

                    shared["auto_step_start"] = now
                    shared["auto_step_end"] = now + duration
                    shared["auto_step_initialized"] = True

                    if command == "M":
                        shared["auto_m_values"] = list(values)
                    elif command == "A":
                        shared["auto_a_values"] = list(values)

                    shared["auto_command"] = command or ""
                    shared["auto_values"] = list(values) if command else []
                    shared["auto_step"] = idx
                    shared["auto_time_remaining"] = max(0.0, shared["auto_step_end"] - now)

                    if duration == 0.0:
                        shared["auto_step"] = idx + 1
                        shared["auto_step_initialized"] = False
                        continue_loop = True
                    else:
                        result = (
                            list(shared.get("auto_m_values", [0, 0])),
                            list(shared.get("auto_a_values", [0, 0, 0, 0, 0, 0])),
                        )
                else:
                    step_end = shared.get("auto_step_end", now)
                    if now >= step_end:
                        shared["auto_step"] = idx + 1
                        shared["auto_step_initialized"] = False
                        continue_loop = True
                    else:
                        shared["auto_time_remaining"] = max(0.0, step_end - now)
                        result = (
                            list(shared.get("auto_m_values", [0, 0])),
                            list(shared.get("auto_a_values", [0, 0, 0, 0, 0, 0])),
                        )

        if stop_reason:
            _stop_auto(shared, stop_reason)
            return None
        if continue_loop:
            continue
        return result


def _sender(shared: dict, stop_event: threading.Event) -> None:
    period = 1.0 / SEND_RATE_HZ
    next_send = time.monotonic()
    link: Optional[serialCommands] = None
    last_servo_sent: Optional[int] = None
    last_servo_send_time = 0.0
    last_dpad_up_pressed = False
    try:
        while not stop_event.is_set():
            now = time.monotonic()
            sleep_for = next_send - now
            if sleep_for > 0:
                stop_event.wait(sleep_for)
                continue
            next_send = now + period

            if link is None:
                link = _open_link(shared, stop_event)
                if link is None:
                    break

            _maybe_start_auto(shared, now)
            auto_outputs = _update_auto_state(shared, now)
            with shared["lock"]:
                axes_snapshot = shared.get("axes", (0, 0, 0, 0, 0, 0, 0, 0))
                buttons_snapshot = shared.get("buttons", tuple())
                vr_angle = int(shared.get("servo_angle", SERVO_CENTER))
                vr_last = float(shared.get("vr_last_update", 0.0))
                vr_status = str(shared.get("vr_status", "idle"))
            dpad_x_raw = int(axes_snapshot[DPAD_X_AXIS_INDEX]) if len(axes_snapshot) > DPAD_X_AXIS_INDEX else 0
            dpad_y_raw = int(axes_snapshot[DPAD_Y_AXIS_INDEX]) if len(axes_snapshot) > DPAD_Y_AXIS_INDEX else 0
            dpad_up_button_pressed = bool(buttons_snapshot[DPAD_UP_BUTTON_INDEX]) if len(buttons_snapshot) > DPAD_UP_BUTTON_INDEX else False

            if auto_outputs is None:
                with shared["lock"]:
                    axes = shared["axes"]
                    armed_effective = shared.get("armed_effective", False)
                    last_udp = shared["last_udp"]
                    shared["control_mode"] = "manual"
                    limiter: Optional[AccelLimiter] = shared.get("drive_limiter")  # type: ignore[assignment]

                throttle = clamp_i8(int(axes[3] / 4))
                steer = clamp_i8(int(axes[4] / 4))
                left = clamp_i8(throttle + steer)
                right = clamp_i8(throttle - steer)
                left = _scale_to_drive(left)
                right = _scale_to_drive(right)
                arm = clamp_i8(int(axes[0] / 4))
                bucket = clamp_i8(int(axes[1] / 4))

                send_dt = now - last_udp
                stale = send_dt > 0.1
                outputs_enabled = (not stale) and armed_effective
                if not outputs_enabled:
                    left = right = arm = bucket = 0
                    if limiter:
                        limiter.reset(0.0, 0.0)
                elif limiter:
                    left_f, right_f = limiter.update(float(left), float(right), period)
                    left = clamp_drive(int(round(left_f)))
                    right = clamp_drive(int(round(right_f)))

                send_m = [right, -left]
                send_a = [-1, -1, -1, -1, arm, -bucket]

                with shared["lock"]:
                    shared["motor_left"] = left
                    shared["motor_right"] = right
                    shared["arm_cmd"] = arm
                    shared["bucket_cmd"] = bucket
                    shared["stale"] = stale
                    shared["outputs_enabled"] = outputs_enabled
                    shared["last_send_dt"] = send_dt
            else:
                send_m, send_a = auto_outputs
                send_m = list(send_m)
                send_a = list(send_a)
                if not send_m:
                    send_m = [0, 0]
                elif len(send_m) == 1:
                    send_m.append(0)
                if len(send_m) < 2:
                    send_m = (send_m + [0, 0])[:2]
                if len(send_a) < 6:
                    send_a = (send_a + [-1, -1, -1, -1, 0, 0])[:6]

                send_m = [clamp_drive(int(v)) for v in send_m]

                with shared["lock"]:
                    shared["stale"] = False
                    shared["outputs_enabled"] = True
                    shared["last_send_dt"] = 0.0
                    shared["motor_left"] = send_m[0]
                    shared["motor_right"] = send_m[1]
                    shared["arm_cmd"] = send_a[4]
                    shared["bucket_cmd"] = send_a[5]
                    shared["control_mode"] = "auto"
                    limiter = shared.get("drive_limiter")
                    if limiter:
                        limiter.reset(0.0, 0.0)

            # Servo/headset traffic is intentionally independent from F310 arming
            # and UDP freshness so camera pan can keep running on its own.
            dpad_up_pressed = dpad_up_button_pressed or (dpad_y_raw <= -DPAD_ACTIVE_THRESHOLD)
            if dpad_up_pressed and not last_dpad_up_pressed:
                vr_angle = SERVO_CENTER
                vr_last = now
                vr_status = "manual center"
                with shared["lock"]:
                    shared["servo_angle"] = vr_angle
                    shared["vr_last_update"] = now
                    shared["vr_status"] = vr_status
                    shared["vr_yaw_deg"] = None
            if abs(dpad_x_raw) >= DPAD_ACTIVE_THRESHOLD:
                delta = (dpad_x_raw / 127.0) * DPAD_SERVO_SPEED_DEG_PER_SEC * period
                vr_angle = _clamp_servo(int(round(vr_angle + delta)))
                vr_last = now
                vr_status = "manual dpad"
                with shared["lock"]:
                    shared["servo_angle"] = vr_angle
                    shared["vr_last_update"] = now
                    shared["vr_status"] = vr_status
                    shared["vr_yaw_deg"] = None
            last_dpad_up_pressed = dpad_up_pressed
            vr_fresh = (now - vr_last) <= HEADSET_STALE_SEC
            servo_to_send: Optional[int] = _clamp_servo(int(vr_angle)) if vr_fresh else None
            vr_status_out = vr_status
            if not vr_status_out.startswith("vr disabled"):
                vr_status_out = "receiving" if vr_fresh else "stale"
            with shared["lock"]:
                shared["vr_connected"] = vr_fresh
                shared["vr_status"] = vr_status_out
                if servo_to_send is not None:
                    shared["servo_cmd"] = servo_to_send

            try: 
                link.send_command("M", send_m)
                link.send_command("A", send_a)
                if servo_to_send is not None:
                    if (
                        last_servo_sent != servo_to_send
                        or (now - last_servo_send_time) >= SERVO_RESEND_INTERVAL
                    ):
                        link.send_command("S", [servo_to_send])
                        last_servo_sent = servo_to_send
                        last_servo_send_time = now
                lines = link.read_serial()
            except Exception as exc:
                with shared["lock"]:
                    shared["serial_status"] = f"Serial send failed: {exc}"
                try:
                    if link is not None:
                        link.close_serial()
                except Exception:
                    pass
                link = None
                last_servo_sent = None
                last_servo_send_time = 0.0
                next_send = time.monotonic() + period
                continue

            if lines:
                last_line = lines[-1]
                if not isinstance(last_line, str):
                    last_line = repr(last_line)
                with shared["lock"]:
                    shared["serial_rx"] = last_line
    finally:
        if link is not None:
            try:
                link.close_serial()
            except Exception:
                pass
        with shared["lock"]:
            shared["serial_status"] = "disconnected"


def _render_display(shared: dict) -> str:
    with shared["lock"]:
        axes = shared.get("axes", (0, 0, 0, 0, 0, 0))
        buttons = shared.get("buttons", tuple())
        armed = bool(shared.get("armed_effective", shared.get("armed", False)))
        net_dt = float(shared.get("net_dt", 0.0))
        last_send_dt = float(shared.get("last_send_dt", 0.0))
        stale = bool(shared.get("stale", True))
        outputs_enabled = bool(shared.get("outputs_enabled", False))
        motor_left = int(shared.get("motor_left", 0))
        motor_right = int(shared.get("motor_right", 0))
        arm_cmd = int(shared.get("arm_cmd", 0))
        bucket_cmd = int(shared.get("bucket_cmd", 0))
        serial_status = str(shared.get("serial_status", "unknown"))
        serial_rx = shared.get("serial_rx", "")
        control_mode = str(shared.get("control_mode", "manual"))
        auto_active = bool(shared.get("auto_active", False))
        auto_step = int(shared.get("auto_step", 0))
        auto_total = int(shared.get("auto_total_steps", 0))
        auto_time = float(shared.get("auto_time_remaining", 0.0))
        auto_command = str(shared.get("auto_command", ""))
        auto_values = list(shared.get("auto_values", []))
        auto_m_values = list(shared.get("auto_m_values", []))
        auto_a_values = list(shared.get("auto_a_values", []))
        auto_program = str(shared.get("auto_program", ""))
        vr_status = str(shared.get("vr_status", "idle"))
        vr_connected = bool(shared.get("vr_connected", False))
        vr_yaw = shared.get("vr_yaw_deg")
        servo_cmd = shared.get("servo_cmd", None)

    axes_str = " ".join(f"{val:4d}" for val in axes)
    if not buttons:
        buttons_str = "(no buttons)"
        pressed_str = "none"
    else:
        buttons_str = " ".join(str(val) for val in buttons)
        pressed = [str(idx) for idx, value in enumerate(buttons) if value]
        pressed_str = ",".join(pressed) if pressed else "none"

    serial_status_display = (
        serial_status if len(serial_status) <= 64 else serial_status[:61] + "..."
    )
    serial_rx_display = (
        serial_rx if isinstance(serial_rx, str) else str(serial_rx)
    )
    if len(serial_rx_display) > 64:
        serial_rx_display = serial_rx_display[:61] + "..."

    lines = [
        "F310 Monitor",
        f"Mode: {control_mode.upper():5}  Prog: {(auto_program or '-'):<10}  Armed: {int(armed)}  Outputs: {'ON ' if outputs_enabled else 'OFF'}  Stale: {int(stale)}",
        f"Net dt: {net_dt:6.3f}s  Send dt: {last_send_dt:6.3f}s",
        f"Axes : {axes_str}",
        f"Btns : {buttons_str}",
        f"Pressed buttons: {pressed_str}",
        f"Motor: left={motor_left:4d} right={motor_right:4d}",
        f"Arm  : arm={arm_cmd:4d} bucket={bucket_cmd:4d}",
        f"Serial: {serial_status_display}",
    ]
    if auto_active or control_mode == "auto":
        step_total = max(1, auto_total)
        current_step = min(auto_step + 1, step_total)
        lines.append(
            f"Auto step: {current_step}/{step_total}  time_left={auto_time:4.1f}s  cmd={auto_command or '-'}"
        )
        values_str = " ".join(str(v) for v in auto_values) if auto_values else "-"
        lines.append(f"Auto values: {values_str}")
        lines.append(
            "Auto M: " + (" ".join(str(v) for v in auto_m_values) if auto_m_values else "-")
        )
        lines.append(
            "Auto A: " + (" ".join(str(v) for v in auto_a_values) if auto_a_values else "-")
        )
    if serial_rx_display:
        lines.append(f"Serial RX: {serial_rx_display}")
    vr_line = f"Headset: {vr_status} ({'link' if vr_connected else 'no-link'})"
    if vr_yaw is not None:
        vr_line += f" yaw={vr_yaw:.1f}deg"
    if servo_cmd is not None:
        vr_line += f" servo={servo_cmd}"
    lines.append(vr_line)
    return "\n".join(lines)


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(LISTEN_ADDR)
    ds_tracker = DriverStationTracker()
    video_settings = VideoSettings(
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        fps=CAMERA_FPS,
        quality=CAMERA_JPEG_QUALITY,
        device_index=CAMERA_DEVICE_INDEX,
        low_latency=CAMERA_LOW_LATENCY,
    )
    capture = VideoCaptureWorker(video_settings)
    bind_host = auto_local_ip()
    http_server = CameraHTTPServer((bind_host, CAMERA_HTTP_PORT), capture)
    http_thread = threading.Thread(target=http_server.serve_forever, name="CameraHTTP", daemon=True)

    shared = {
        "axes": (0, 0, 0, 0, 0, 0),
        "buttons": tuple(0 for _ in range(12)),
        "armed": False,
        "armed_effective": False,
        "armed_miss_count": 0,
        "last_udp": 0.0,
        "net_dt": 0.0,
        "last_send_dt": 0.0,
        "stale": True,
        "outputs_enabled": False,
        "motor_left": 0,
        "motor_right": 0,
        "arm_cmd": 0,
        "bucket_cmd": 0,
        "serial_status": "disconnected",
        "serial_rx": "",
        "control_mode": "manual",
        "auto_requested": None,
        "auto_active": False,
        "auto_sequence": [],
        "auto_total_steps": 0,
        "auto_step": 0,
        "auto_step_start": 0.0,
        "auto_step_end": 0.0,
        "auto_step_initialized": False,
        "auto_time_remaining": 0.0,
        "auto_command": "",
        "auto_values": [],
        "auto_m_values": [0, 0],
        "auto_a_values": [0, 0, 0, 0, 0, 0],
        "auto_program": "",
        "vr_status": "idle",
        "vr_connected": False,
        "vr_last_update": 0.0,
        "vr_yaw_deg": None,
        "servo_angle": SERVO_CENTER,
        "servo_cmd": None,
        "lock": threading.Lock(),
        "drive_limiter": AccelLimiter(
            ACCEL_LIMIT_PER_SEC,
            DECEL_LIMIT_PER_SEC,
            DECEL_NEAR_ZERO_PER_SEC,
            DECEL_NEAR_ZERO_THRESHOLD,
        ),
    }
    stop_event = threading.Event()
    discovery_thread = threading.Thread(
        target=_discovery_responder, args=(ds_tracker, stop_event), daemon=True
    )
    telemetry = TelemetryBroadcaster(ds_tracker, interval=1.0 / TELEMETRY_RATE_HZ)

    headset_thread = threading.Thread(
        target=_headset_listener, args=(shared, stop_event), daemon=True
    )

    rx_thread = threading.Thread(
        target=_receiver, args=(sock, ds_tracker, shared, stop_event), daemon=True
    )
    tx_thread = threading.Thread(
        target=_sender, args=(shared, stop_event), daemon=True
    )
    capture.start()
    http_thread.start()
    print(
        f"[video] Serving JPEG snapshots on {bind_host}:{CAMERA_HTTP_PORT}/frame "
        f"(device {video_settings.device_index if video_settings.device_index >= 0 else 'auto'}, "
        f"{video_settings.width}x{video_settings.height}@{video_settings.fps}fps, q={video_settings.quality})"
    )
    discovery_thread.start()
    telemetry.start()
    headset_thread.start()
    rx_thread.start()
    tx_thread.start()

    try:
        sys.stdout.write("\033[2J")
        sys.stdout.flush()
        while not stop_event.is_set():
            if not rx_thread.is_alive() or not tx_thread.is_alive():
                break
            display = _render_display(shared)
            sys.stdout.write("\033[H" + display + "\n\033[0J")
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        with shared["lock"]:
            shared["serial_status"] = "Interrupted by user"
        stop_event.set()
    finally:
        stop_event.set()
        rx_thread.join(timeout=1.0)
        tx_thread.join(timeout=1.0)
        discovery_thread.join(timeout=1.0)
        telemetry.stop()
        headset_thread.join(timeout=1.0)
        http_server.shutdown()
        http_thread.join(timeout=2.0)
        capture.stop()
        sock.close()
        display = _render_display(shared)
        sys.stdout.write("\033[H" + display + "\n\033[0J\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
