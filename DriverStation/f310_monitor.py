#!/usr/bin/env python3
"""Bridge F310 UDP control packets to the Arduino serial protocol.

This script listens for joystick packets produced by ``f310_send.py`` (or the GUI)
via UDP and forwards the decoded commands to the Arduino using the
``serial_format.txt`` framing.  It performs a simple tank-drive mix for the drive
motors and exposes configurable axis/button mappings for actuators, servo, and
LEDs.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple
from typing import Iterable, Optional, Sequence, Tuple

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None  # type: ignore

from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

try:
    from serial import SerialException  # type: ignore
except Exception:  # pragma: no cover - serial may be unavailable during dry runs
    SerialException = Exception  # type: ignore

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from system_operations.arduino_serial_commuication import serialCommands
except Exception as exc:  # pragma: no cover - import failure handled later
    serialCommands = None  # type: ignore
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

try:
    from system_operations.arduino_cli import arduinoConsole
except Exception as exc:  # pragma: no cover - optional dependency
    arduinoConsole = None  # type: ignore
    ARDUINO_CONSOLE_ERROR = exc
else:
    ARDUINO_CONSOLE_ERROR = None

SYNC_FULL_STATE = 0xA6
AXIS_COUNT = 6
LED_COUNT = 4


@dataclass
class ControlState:
    seq: int
    armed: bool
    axes: Tuple[int, ...]
    buttons_mask: int


@dataclass
class ControlMapping:
    throttle_axis: int
    steer_axis: Optional[int]
    arm_axis: Optional[int]
    bucket_axis: Optional[int]
    servo_button: Optional[int]
    led_buttons: Optional[Tuple[Optional[int], Optional[int], Optional[int], Optional[int]]]
    deadband: int
    invert_throttle: bool
    invert_steer: bool
    invert_arm: bool
    invert_bucket: bool
    motor_max_speed: int


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class SimpleCameraServer:
    """Minimal camera server that exposes snapshot and MJPEG endpoints."""

    def __init__(
        self,
        source: Optional[str],
        host: str,
        port: int,
        fps: float,
        max_width: Optional[int],
        jpeg_quality: int,
    ) -> None:
        self._raw_source = None if source is None else source.strip()
        self._host = host
        self._port = port
        self._capture: Optional["cv2.VideoCapture"] = None  # type: ignore[name-defined]
        self._frame_lock = threading.Lock()
        self._frame: Optional[bytes] = None
        self._stop_event = threading.Event()
        self._capture_thread: Optional[threading.Thread] = None
        self._httpd: Optional[ThreadedHTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None
        self._frame_interval = 0.0
        if fps > 0:
            self._frame_interval = 1.0 / fps
        self._max_width = max_width if max_width and max_width > 0 else None
        self._jpeg_quality = min(95, max(10, jpeg_quality))

    def start(self) -> Optional[str]:
        if self._raw_source is None:
            return None
        if cv2 is None:
            print("OpenCV is not available; camera streaming disabled.")
            return None
        try:
            source: object = int(self._raw_source)
        except (TypeError, ValueError):
            source = self._raw_source
        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            print(f"Failed to open camera source {self._raw_source}")
            capture.release()
            return None
        self._capture = capture
        self._stop_event.clear()
        self._capture_thread = threading.Thread(target=self._capture_loop, name="CameraCapture", daemon=True)
        self._capture_thread.start()

        server = self

        class Handler(BaseHTTPRequestHandler):  # pragma: no cover - simple HTTP handler
            def do_GET(self) -> None:
                if self.path in {"/", "/frame"}:
                    frame = server._get_frame()
                    if frame is None:
                        self.send_error(503, "Camera frame unavailable")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                    self.end_headers()
                    boundary = b"--FRAME\r\n"
                    try:
                        while not server._stop_event.is_set():
                            frame = server._get_frame()
                            if frame is None:
                                time.sleep(0.05)
                                continue
                            self.wfile.write(boundary)
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                            if server._frame_interval > 0:
                                time.sleep(server._frame_interval)
                    except BrokenPipeError:
                        return
                else:
                    self.send_error(404, "Not Found")

            def log_message(self, *_args: object) -> None:
                return

        try:
            httpd = ThreadedHTTPServer((self._host, self._port), Handler)
        except OSError as exc:
            print(f"Failed to start camera server on {self._host}:{self._port}: {exc}")
            self._stop_event.set()
            if self._capture is not None:
                self._capture.release()
                self._capture = None
            return None

        self._httpd = httpd
        self._http_thread = threading.Thread(target=httpd.serve_forever, name="CameraHTTP", daemon=True)
        self._http_thread.start()
        url_host = "127.0.0.1" if self._host == "0.0.0.0" else self._host
        url = f"http://{url_host}:{self._port}/frame"
        print(f"Camera feed available at {url}")
        return url

    def stop(self) -> None:
        self._stop_event.set()
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:
                pass
            self._httpd.server_close()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
        if self._http_thread is not None:
            self._http_thread.join(timeout=1.0)
        if self._capture is not None:
            self._capture.release()
        self._capture = None
        self._httpd = None
        self._capture_thread = None
        self._http_thread = None
        self._frame = None

    def _capture_loop(self) -> None:
        assert self._capture is not None
        while not self._stop_event.is_set():
            ret, frame = self._capture.read()
            if not ret:
                time.sleep(0.1)
                continue
            if self._max_width is not None:
                height, width = frame.shape[:2]
                if width > self._max_width:
                    scale = self._max_width / float(width)
                    new_size = (self._max_width, max(1, int(height * scale)))
                    frame = cv2.resize(frame, new_size)  # type: ignore[arg-type]
            quality_flag = getattr(cv2, "IMWRITE_JPEG_QUALITY", 1)
            encode_params = [int(quality_flag), int(self._jpeg_quality)]
            ok, buffer = cv2.imencode('.jpg', frame, encode_params)
            if not ok:
                continue
            with self._frame_lock:
                self._frame = buffer.tobytes()
            if self._frame_interval > 0:
                time.sleep(self._frame_interval)

    def _get_frame(self) -> Optional[bytes]:
        with self._frame_lock:
            return self._frame


class TelemetryGenerator:
    """Produces synthetic telemetry data for the driver station."""

    def __init__(self) -> None:
        self._phase = 0.0

    def sample(self) -> dict:
        self._phase = (self._phase + 0.12) % (2 * math.pi)
        phase = self._phase
        temps = [25.0 + 4.5 * math.sin(phase + i * 0.4) for i in range(6)]
        strain = [math.sin(phase * 1.5 + i * 0.6) for i in range(6)]
        hall = [45.0 + 6.0 * math.cos(phase + i * 0.5) for i in range(6)]
        actuator = 35.0 + 28.0 * math.sin(phase * 0.8)
        voltage = 12.2 + 0.3 * math.sin(phase * 0.3)
        current = 2.4 + 0.4 * math.cos(phase * 0.5)
        payload = {
            "timestamp": time.time(),
            "voltage": round(voltage, 2),
            "current": round(current, 2),
            "temperature": round(temps[0], 2),
            "actuator_position": round(actuator, 2),
            "temperatures": [round(t, 2) for t in temps],
            "strain": [round(s, 3) for s in strain],
            "hall": [round(h, 2) for h in hall],
        }
        return payload

<<<<<<< ours
=======
    def set_mode(self, mode: Optional[str]) -> None:
        self._mode = mode or "manual"


class AutoRunner:
    """Loads and executes predefined autonomous scripts."""

    def __init__(
        self,
        send_serial: Callable[[str, Iterable[int]], None],
        telemetry: TelemetryGenerator,
        maybe_send_dummy: Callable[[bool], None],
    ) -> None:
        self._send_serial = send_serial
        self._telemetry = telemetry
        self._maybe_send_dummy = maybe_send_dummy
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._active: Optional[str] = None

    def start(self, script: str) -> str:
        script = script.lower()
        with self._lock:
            if self._active == script and self._thread and self._thread.is_alive():
                return f"auto '{script}' already running"
            self.stop()
            module_name = f"DriverStation.autonomous.{script}"
            try:
                module = importlib.import_module(module_name)
            except ModuleNotFoundError:
                return f"unknown auto script '{script}'"
            if hasattr(module, "get_sequence") and callable(module.get_sequence):
                sequence = module.get_sequence()
            elif hasattr(module, "SEQUENCE"):
                sequence = module.SEQUENCE  # type: ignore[attr-defined]
            else:
                return f"script '{script}' missing SEQUENCE"
            if not isinstance(sequence, Iterable):
                return f"script '{script}' returned invalid sequence"
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                args=(script, list(sequence)),
                name=f"AutoRunner-{script}",
                daemon=True,
            )
            self._thread.start()
            self._active = script
            self._telemetry.set_mode(script)
            return f"auto '{script}' started"

    def stop(self) -> str:
        with self._lock:
            thread = self._thread
            if thread is None:
                self._active = None
                self._telemetry.set_mode(None)
                return "no auto run active"
            self._stop_event.set()
        thread.join(timeout=2.0)
        self._stop_event.clear()
        with self._lock:
            self._thread = None
            self._active = None
            self._telemetry.set_mode(None)
        self._send_serial('M', [0, 0])
        self._send_serial('A', [-1, -1, -1, -1, 0, 0])
        self._maybe_send_dummy(True)
        return "auto run stopped"

    def is_active(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def current(self) -> Optional[str]:
        with self._lock:
            return self._active

    def _run(self, script: str, sequence: List[dict]) -> None:
        try:
            for step in sequence:
                if self._stop_event.is_set():
                    break
                command = step.get("command")
                values = step.get("values", [])
                duration = float(step.get("duration", 0.0))
                if command:
                    try:
                        self._send_serial(command, values)
                    except Exception as exc:
                        print(f"Auto '{script}' send failed: {exc}")
                        break
                    self._maybe_send_dummy(True)
                if duration > 0:
                    end = time.monotonic() + duration
                    while not self._stop_event.is_set() and time.monotonic() < end:
                        remaining = end - time.monotonic()
                        time.sleep(min(0.1, max(0.01, remaining)))
                        self._maybe_send_dummy(True)
        finally:
            self._send_serial('M', [0, 0])
            self._send_serial('A', [-1, -1, -1, -1, 0, 0])
            self._maybe_send_dummy(True)
            with self._lock:
                self._thread = None
                self._active = None
                self._telemetry.set_mode(None)
            self._stop_event.clear()

>>>>>>> theirs

def clamp_int8(value: int) -> int:
    return max(-128, min(127, int(value)))


def apply_deadband(value: int, threshold: int) -> int:
    if abs(value) <= threshold:
        return 0
    return value


def get_axis(axes: Sequence[int], index: Optional[int], *, invert: bool, deadband: int) -> int:
    if index is None or index < 0 or index >= len(axes):
        return 0
    value = axes[index]
    if invert:
        value = -value
    value = apply_deadband(value, deadband)
    if value < 0 and value <= -127:
        return -128
    return value


def button_pressed(mask: int, index: Optional[int]) -> bool:
    if index is None or index < 0:
        return False
    if index >= 16:
        return False
    return bool(mask & (1 << index))


def decode_packet(data: bytes) -> Optional[ControlState]:
    if not data:
        return None
    pkt_type = data[0]
    if pkt_type != SYNC_FULL_STATE:
        return None
    if len(data) < 12:
        raise ValueError("packet too short for full-state frame")
    checksum = 0
    for b in data[:-1]:
        checksum ^= b
    if checksum != data[-1]:
        raise ValueError("checksum mismatch")
    seq = data[1]
    flags = data[2]
    axes = []
    for i in range(AXIS_COUNT):
        raw = data[3 + i]
        axes.append(struct.unpack('b', bytes((raw,)))[0])
    buttons_mask = data[9] | (data[10] << 8)
    armed = bool(flags & 0x01)
    return ControlState(seq=seq, armed=armed, axes=tuple(axes), buttons_mask=buttons_mask)


def compute_motor_outputs(state: ControlState, mapping: ControlMapping) -> Tuple[int, int]:
    throttle = get_axis(
        state.axes,
        mapping.throttle_axis,
        invert=mapping.invert_throttle,
        deadband=mapping.deadband,
    )
    steer = get_axis(
        state.axes,
        mapping.steer_axis,
        invert=mapping.invert_steer,
        deadband=mapping.deadband,
    )
    left_raw = clamp_int8(throttle + steer)
    right_raw = clamp_int8(throttle - steer)
    max_speed = max(1, min(127, mapping.motor_max_speed))
    if max_speed >= 127:
        left = left_raw
        right = right_raw
    else:
        scale = max_speed / 127.0
        left = clamp_int8(int(round(left_raw * scale)))
        right = clamp_int8(int(round(right_raw * scale)))
    if not state.armed:
        left = 0
        right = 0
    return left, right


def compute_actuator_outputs(state: ControlState, mapping: ControlMapping) -> Optional[Tuple[int, int]]:
    if mapping.arm_axis is None and mapping.bucket_axis is None:
        return None
    arm_vel = get_axis(
        state.axes,
        mapping.arm_axis,
        invert=mapping.invert_arm,
        deadband=mapping.deadband,
    )
    bucket_vel = get_axis(
        state.axes,
        mapping.bucket_axis,
        invert=mapping.invert_bucket,
        deadband=mapping.deadband,
    )
    if not state.armed:
        arm_vel = 0
        bucket_vel = 0
    return arm_vel, bucket_vel


def compute_servo_output(state: ControlState, mapping: ControlMapping) -> Optional[int]:
    if mapping.servo_button is None:
        return None
    return 1 if button_pressed(state.buttons_mask, mapping.servo_button) else 0


def compute_led_output(state: ControlState, mapping: ControlMapping) -> Tuple[int, int, int, int]:
    if mapping.led_buttons is None:
        return (1 if not state.armed else 0, 0, 1 if state.armed else 0, 0)
    red_idx, yellow_idx, green_idx, blue_idx = mapping.led_buttons
    return (
        1 if button_pressed(state.buttons_mask, red_idx) else 0,
        1 if button_pressed(state.buttons_mask, yellow_idx) else 0,
        1 if button_pressed(state.buttons_mask, green_idx) else 0,
        1 if button_pressed(state.buttons_mask, blue_idx) else 0,
    )


def run_server(args: argparse.Namespace) -> None:
    host, port = parse_host_port(args.listen)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((host, port))
    except OSError as exc:
        raise SystemExit(f"Failed to bind UDP socket on {host}:{port}: {exc}")
    sock.settimeout(0.2)
    print(f"Listening for F310 packets on {host}:{port}")

    deadband = max(0, min(int(round(args.deadband * 127.0)), 40))
    mapping = ControlMapping(
        throttle_axis=args.throttle_axis,
        steer_axis=None if args.steer_axis < 0 else args.steer_axis,
        arm_axis=None if args.arm_axis < 0 else args.arm_axis,
        bucket_axis=None if args.bucket_axis < 0 else args.bucket_axis,
        servo_button=None if args.servo_button < 0 else args.servo_button,
        led_buttons=parse_led_buttons(args.led_buttons),
        deadband=deadband,
        invert_throttle=args.invert_throttle,
        invert_steer=args.invert_steer,
        invert_arm=args.invert_arm,
        invert_bucket=args.invert_bucket,
        motor_max_speed=max(1, min(127, args.motor_max_speed)),
    )

    camera_server: Optional[SimpleCameraServer] = None
    if args.camera_source:
        camera_server = SimpleCameraServer(
            args.camera_source,
            args.camera_host,
            args.camera_port,
            args.camera_fps,
            args.camera_width,
            args.camera_quality,
        )
        camera_server.start()

    telemetry_sock: Optional[socket.socket] = None
    telemetry_dest: Optional[Tuple[str, int]] = None
    telemetry_interval = 1.0 / max(0.1, args.telemetry_rate)
    telemetry_next = time.monotonic()
    telemetry_gen = TelemetryGenerator()
    if args.telemetry_dest:
        telemetry_dest = parse_host_port(args.telemetry_dest)
        telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        telemetry_sock.setblocking(False)
        initial_payload = telemetry_gen.sample()
        try:
            telemetry_sock.sendto(json.dumps(initial_payload).encode("utf-8"), telemetry_dest)
        except OSError:
            pass
        telemetry_next = time.monotonic() + telemetry_interval

    if args.upload:
        sketch_path = ROOT / "system_operations" / "system_control" / "system_control.ino"
        if arduinoConsole is None:
            raise SystemExit(
                "--upload requested but arduinoConsole unavailable: {}".format(
                    ARDUINO_CONSOLE_ERROR
                )
            )
        try:
            console = arduinoConsole(sketch_path=sketch_path)  # type: ignore[call-arg]
            console.compile_and_upload()
        except Exception as exc:  # pragma: no cover - depends on hardware
            print(f"Arduino upload failed: {exc}")

    serial_state: dict[str, object] = {"serial": None, "next_retry": 0.0}
    max_rate_hz = 100.0
    min_interval = 1.0 / max_rate_hz
    last_send_time = 0.0

    def maybe_send_dummy(force: bool = False) -> None:
        nonlocal telemetry_next
        if telemetry_sock is None or telemetry_dest is None:
            return
        now = time.monotonic()
        if not force and now < telemetry_next:
            return
        payload = telemetry_gen.sample()
        try:
            telemetry_sock.sendto(json.dumps(payload).encode("utf-8"), telemetry_dest)
        except OSError:
            pass
        telemetry_next = now + telemetry_interval

    def ensure_serial() -> bool:
        if SERIAL_IMPORT_ERROR is not None:
            return False
        ser = serial_state["serial"]
        if ser is not None:
            return True
        now = time.monotonic()
        if now < serial_state["next_retry"]:
            return False
        try:
            serial_state["serial"] = serialCommands(  # type: ignore[call-arg]
                port=args.serial_port,
                baudrate=args.baudrate,
            )
            ser = serial_state["serial"]
            try:
                port_name = getattr(ser, "port", args.serial_port)
            except Exception:
                port_name = args.serial_port
            print(f"Connected to Arduino on {port_name or 'auto-detected'}")
            return True
        except (SerialException, RuntimeError, OSError) as exc:
            print(f"Serial connect failed: {exc}")
            serial_state["serial"] = None
            serial_state["next_retry"] = now + 2.0
            return False

    def close_serial() -> None:
        ser = serial_state.get("serial")
        if ser is not None:
            try:
                ser.close_serial()
            except Exception:
                pass
        serial_state["serial"] = None

    def send_serial(command: str, values: Iterable[int]) -> None:
        nonlocal last_send_time
        now = time.monotonic()
        elapsed = now - last_send_time
        if elapsed < min_interval:
            time.sleep(min_interval - elapsed)
            now = time.monotonic()
        last_send_time = now
        int8_values = [clamp_int8(v) for v in values]
        connected = ensure_serial()
        label = "ok" if connected else "pending"
        print(f"[serial {label}] {command} int8={int8_values}")
        if not connected:
            return
        ser = serial_state.get("serial")
        if ser is None:
            return
        payload = [(v + 256) % 256 for v in int8_values]
        try:
            ser.send_command(command, payload)
        except (SerialException, OSError) as exc:
            print(f"Serial write failed: {exc}")
            serial_state["next_retry"] = time.monotonic() + 1.0
            close_serial()

    last_packet_time = 0.0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(256)
            except socket.timeout:
                if args.timeout > 0 and last_packet_time > 0.0:
                    if time.monotonic() - last_packet_time >= args.timeout:
                        idle_state = ControlState(seq=0, armed=False, axes=(0,) * AXIS_COUNT, buttons_mask=0)
                        left, right = compute_motor_outputs(idle_state, mapping)
                        send_serial('M', (right, left))
                        last_packet_time = 0.0
                maybe_send_dummy()
                continue
            except KeyboardInterrupt:
                print("Interrupted; exiting")
                break

            try:
                state = decode_packet(data)
            except ValueError as exc:
                print(f"Ignored malformed packet: {exc}")
                continue
            if state is None:
                continue

            last_packet_time = time.monotonic()
            print(
                f"[packet] seq={state.seq} armed={int(state.armed)} axes={list(state.axes)} "
                f"buttons=0x{state.buttons_mask:04X}",
                flush=True,
            )
<<<<<<< ours
<<<<<<< ours
<<<<<<< ours
=======
            if auto_runner.is_active():
                maybe_send_dummy()
                continue
<<<<<<< ours
>>>>>>> theirs
=======
>>>>>>> theirs
=======
>>>>>>> theirs
=======
            if auto_runner.is_active():
<<<<<<< ours
                if state.armed or state.buttons_mask:
                    print("[command] manual override requested; stopping auto")
                    auto_runner.stop()
                else:
                    maybe_send_dummy()
                    continue
>>>>>>> theirs
=======
                maybe_send_dummy()
                continue
>>>>>>> theirs
            left, right = compute_motor_outputs(state, mapping)
            send_serial('M', (right, left))
            actuator = compute_actuator_outputs(state, mapping)
            if actuator is not None:
                arm_vel, bucket_vel = actuator
                send_serial('A', (-1, -1, -1, -1, arm_vel, bucket_vel))
            maybe_send_dummy()

    finally:
        close_serial()
        sock.close()
        if camera_server is not None:
            camera_server.stop()
        if telemetry_sock is not None:
            try:
                telemetry_sock.close()
            except OSError:
                pass


def parse_host_port(value: str) -> Tuple[str, int]:
    if ':' not in value:
        raise SystemExit("listen address must be in host:port format")
    host, port_s = value.rsplit(':', 1)
    if not host:
        host = '0.0.0.0'
    try:
        port = int(port_s)
    except ValueError:
        raise SystemExit(f"Invalid port value: {port_s}")
    return host, port


def parse_led_buttons(values: Optional[Sequence[int]]) -> Optional[Tuple[Optional[int], Optional[int], Optional[int], Optional[int]]]:
    if values is None:
        return None
    if len(values) != LED_COUNT:
        raise SystemExit("--led-buttons requires exactly four integers")
    processed: Tuple[Optional[int], Optional[int], Optional[int], Optional[int]] = tuple(
        (None if v < 0 else v) for v in values
    )  # type: ignore[assignment]
    return processed


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Forward F310 UDP packets to Arduino over serial")
    parser.add_argument(
        "--listen",
        default="0.0.0.0:11000",
        help="host:port to bind for UDP control (default 0.0.0.0:11000)",
    )
    parser.add_argument("--serial-port", default=None, help="Explicit serial port for Arduino (auto-detect if omitted)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Arduino serial baud rate (default 115200)")
    parser.add_argument("--deadband", type=float, default=0.08, help="Axis deadband in [0,1] before command becomes zero")
    parser.add_argument("--timeout", type=float, default=0.5, help="Seconds before sending idle commands when link lost")
    parser.add_argument("--throttle-axis", type=int, default=1, help="Axis index for forward/back (default 1)")
    parser.add_argument("--steer-axis", type=int, default=0, help="Axis index for steering mix (-1 to disable)")
    parser.add_argument("--motor-max-speed", type=int, default=30, help="Max magnitude sent for motor commands (int8 range)")
    parser.add_argument("--arm-axis", type=int, default=3, help="Axis index for arm velocity (-1 to disable)")
    parser.add_argument("--bucket-axis", type=int, default=4, help="Axis index for bucket velocity (-1 to disable)")
    parser.add_argument("--servo-button", type=int, default=-1, help="Button index for servo state (-1 to disable)")
    parser.add_argument(
        "--led-buttons",
        metavar=("RED", "YEL", "GRN", "BLU"),
        nargs=4,
        type=int,
        default=None,
        help="Button indexes controlling LEDs (-1 to ignore)",
    )
    parser.add_argument("--invert-throttle", action="store_true", help="Invert throttle axis")
    parser.add_argument("--invert-steer", action="store_true", help="Invert steer axis")
    parser.add_argument("--invert-arm", action="store_true", help="Invert arm axis")
    parser.add_argument("--invert-bucket", action="store_true", help="Invert bucket axis")
    parser.add_argument("--upload", action="store_true", help="Compile/upload Arduino sketch before connecting")
    parser.add_argument(
        "--camera-source",
        default="0",
        help="OpenCV camera source (index or path) to stream to operators (default 0)",
    )
    parser.add_argument(
        "--camera-host",
        default="0.0.0.0",
        help="Interface to bind the simple camera HTTP server (default 0.0.0.0)",
    )
    parser.add_argument(
        "--camera-port",
        type=int,
        default=8081,
        help="Port for the simple camera HTTP server (default 8081)",
    )
    parser.add_argument(
        "--camera-fps",
        type=float,
        default=6.0,
        help="Capture/update rate for the camera stream (default 6 Hz)",
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=320,
        help="Resize camera frames to this width before encoding (default 320)",
    )
    parser.add_argument(
        "--camera-quality",
        type=int,
        default=45,
        help="JPEG quality (10-95) for the camera stream (default 45)",
    )
    parser.add_argument(
        "--telemetry-dest",
        default="127.0.0.1:10000",
        help="host:port to send telemetry JSON (set empty to disable)",
    )
    parser.add_argument(
        "--telemetry-rate",
        type=float,
        default=2.0,
        help="Telemetry send rate in Hz (default 2)",
    )
    return parser


def main() -> None:
    parser = build_argument_parser()
    args = parser.parse_args()
    run_server(args)


if __name__ == "__main__":
    main()
