#!/usr/bin/env python3
"""Forward F310 GUI UDP packets to the Arduino with minimal processing."""

from __future__ import annotations

import socket
import struct
import sys
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


def _open_link() -> serialCommands:
    while True:
        try:
            return serialCommands()
        except Exception as exc:
            print(f"Arduino not found ({exc}); retrying in 1s...")
            time.sleep(1.0)


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(LISTEN_ADDR)

    link: Optional[serialCommands] = None
    try:
        while True:
            if link is None:
                link = _open_link()
            data, _ = sock.recvfrom(64)
            decoded = decode_packet(data)
            if decoded is None:
                continue
            axes, armed = decoded
            throttle = clamp_i8(int(axes[1]))
            steer = clamp_i8(int(axes[0]))
            left = clamp_i8(throttle + steer)
            right = clamp_i8(throttle - steer)
            arm = clamp_i8(int(axes[3]))
            bucket = clamp_i8(int(axes[4]))
            if not armed:
                left = right = arm = bucket = 0

            try:
                print(f"M: right={right} left={left}")
                link.send_command("M", [right, left])
                print(f"A: arm={arm} bucket={bucket}")
                link.send_command("A", [-1, -1, -1, -1, 0, -bucket])
                try:
                    lines = link.read_serial()
                except Exception as exc:
                    print(f"Serial read failed ({exc})")
                else:
                    if lines:
                        for line in lines:
                            print(f"[Arduino] {line}")
            except Exception as exc:
                print(f"Serial send failed ({exc}); reconnecting...")
                try:
                    link.close_serial()
                except Exception:
                    pass
                link = None
                time.sleep(1.0)
    finally:
        if link is not None:
            link.close_serial()
        sock.close()


if __name__ == "__main__":
    main()
