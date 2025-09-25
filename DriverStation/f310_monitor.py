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
import socket
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple

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
            left, right = compute_motor_outputs(state, mapping)
            send_serial('M', (right, left))

    finally:
        close_serial()
        sock.close()


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
    parser.add_argument("--listen", default="0.0.0.0:9999", help="host:port to bind for UDP control (default 0.0.0.0:9999)")
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
    return parser


def main() -> None:
    parser = build_argument_parser()
    args = parser.parse_args()
    run_server(args)


if __name__ == "__main__":
    main()