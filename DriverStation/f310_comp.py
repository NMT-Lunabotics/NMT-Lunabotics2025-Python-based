#!/usr/bin/env python3
"""Forward F310 GUI UDP packets to the Arduino with minimal processing."""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time
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

SYNC = 0xA6
LISTEN_ADDR: Tuple[str, int] = ("192.168.0.207", 11000)
SEND_RATE_HZ = 50.0

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


def clamp_i8(value: int) -> int:
    return max(-127, min(127, value))


def decode_packet(packet: bytes) -> Optional[Tuple[Tuple[int, ...], Tuple[int, ...], bool]]:
    if len(packet) < 12 or packet[0] != SYNC:
        return None
    checksum = 0
    for byte in packet[:-1]:
        checksum ^= byte
    if checksum != packet[-1]:
        return None
    axes = struct.unpack("bbbbbb", packet[3:9])
    buttons_mask = packet[9] | (packet[10] << 8)
    buttons = tuple((buttons_mask >> bit) & 0x01 for bit in range(12))
    armed = bool(packet[2] & 0x01)
    return axes, buttons, armed


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


def _receiver(sock: socket.socket, shared: dict, stop_event: threading.Event) -> None:
    sock.settimeout(0.2)
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(64)
        except socket.timeout:
            continue
        except OSError:
            break
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

            if auto_outputs is None:
                with shared["lock"]:
                    axes = shared["axes"]
                    armed = shared["armed"]
                    last_udp = shared["last_udp"]
                    shared["control_mode"] = "manual"

                throttle = clamp_i8(int(axes[3] / 4))
                steer = clamp_i8(int(axes[4] / 4))
                left = clamp_i8(throttle + steer)
                right = clamp_i8(throttle - steer)
                arm = clamp_i8(int(axes[0] / 4))
                bucket = clamp_i8(int(axes[1] / 4))

                send_dt = now - last_udp
                stale = send_dt > 0.1
                outputs_enabled = (not stale) and armed
                if not outputs_enabled:
                    left = right = arm = bucket = 0

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

                with shared["lock"]:
                    shared["stale"] = False
                    shared["outputs_enabled"] = True
                    shared["last_send_dt"] = 0.0
                    shared["motor_left"] = send_m[0]
                    shared["motor_right"] = send_m[1]
                    shared["arm_cmd"] = send_a[4]
                    shared["bucket_cmd"] = send_a[5]
                    shared["control_mode"] = "auto"

            try: 
                link.send_command("M", send_m)
                link.send_command("A", send_a)
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
        armed = bool(shared.get("armed", False))
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
    return "\n".join(lines)


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(LISTEN_ADDR)

    shared = {
        "axes": (0, 0, 0, 0, 0, 0),
        "buttons": tuple(0 for _ in range(12)),
        "armed": False,
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
        "lock": threading.Lock(),
    }
    stop_event = threading.Event()

    rx_thread = threading.Thread(
        target=_receiver, args=(sock, shared, stop_event), daemon=True
    )
    tx_thread = threading.Thread(
        target=_sender, args=(shared, stop_event), daemon=True
    )
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
        sock.close()
        display = _render_display(shared)
        sys.stdout.write("\033[H" + display + "\n\033[0J\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
