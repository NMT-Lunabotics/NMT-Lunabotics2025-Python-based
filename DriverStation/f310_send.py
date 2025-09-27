#!/usr/bin/env python3
"""Forward F310 GUI UDP packets to the Arduino with minimal processing."""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from system_operations.arduino_serial_commuication.arduino_serial_commuication import (
    serialCommands,
)

SYNC = 0xA6
LISTEN_ADDR: Tuple[str, int] = ("0.0.0.0", 11000)
SEND_RATE_HZ = 50.0


def clamp_i8(value: int) -> int:
    return max(-127, min(127, value))


def decode_packet(packet: bytes) -> Optional[Tuple[Tuple[int, ...], bool]]:
    if len(packet) < 12 or packet[0] != SYNC:
        return None
    checksum = 0
    for byte in packet[:-1]:
        checksum ^= byte
    if checksum != packet[-1]:
        return None
    axes = struct.unpack("bbbbbb", packet[3:9])
    armed = bool(packet[2] & 0x01)
    return axes, armed


def _open_link(stop_event: threading.Event) -> Optional[serialCommands]:
    while not stop_event.is_set():
        try:
            return serialCommands()
        except Exception as exc:
            print(f"Arduino not found ({exc}); retrying in 1s...")
            stop_event.wait(1.0)
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
        axes, armed = decoded
        now = time.monotonic()
        with shared["lock"]:
            prev = shared["last_udp"]
            shared["axes"] = axes
            shared["armed"] = armed
            shared["last_udp"] = now
        interval = 0.0 if prev == 0.0 else now - prev
        print(f"[net ] dt={interval:.3f}s armed={int(armed)} axes={axes}")


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
                link = _open_link(stop_event)
                if link is None:
                    break

            with shared["lock"]:
                axes = shared["axes"]
                armed = shared["armed"]
                last_udp = shared["last_udp"]

            throttle = clamp_i8(int(axes[3]/4))
            steer = clamp_i8(int(axes[4]/4))
            left = clamp_i8(throttle + steer)
            right = clamp_i8(throttle - steer)
            arm = clamp_i8(int(axes[0]/4))
            bucket = clamp_i8(int(axes[1]/4))

            stale = (now - last_udp) > 0.1
            if stale or not armed:
                left = right = arm = bucket = 0

            try:
                print(
                    f"[send] dt={now - last_udp:.3f}s stale={int(stale)} M: r={right} l={left}"
                )
                link.send_command("M", [right, -left])
                print(f"[send] A: arm={arm} bucket={bucket}")
                link.send_command("A", [-1, -1, -1, -1, 0, -bucket])
                lines = link.read_serial()
                #if lines:
                    #for line in lines:
                        #print(f"[arduino] {line}")
            except Exception as exc:
                print(f"Serial send failed ({exc}); reconnecting...")
                try:
                    if link is not None:
                        link.close_serial()
                except Exception:
                    pass
                link = None
                next_send = time.monotonic() + period
    finally:
        if link is not None:
            try:
                link.close_serial()
            except Exception:
                pass


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(LISTEN_ADDR)

    shared = {
        "axes": (0, 0, 0, 0, 0, 0),
        "armed": False,
        "last_udp": 0.0,
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
        while rx_thread.is_alive() and tx_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted; shutting down")
    finally:
        stop_event.set()
        rx_thread.join(timeout=1.0)
        tx_thread.join(timeout=1.0)
        sock.close()


if __name__ == "__main__":
    main()
