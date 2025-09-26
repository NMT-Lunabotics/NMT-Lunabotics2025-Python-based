#!/usr/bin/env python3
"""Minimal terminal interface for sending commands to the Arduino.

The script opens a ``serialCommands`` connection and presents a prompt.  Every
non-empty line you type is transmitted with the trailing newline expected by the
Arduino firmware.  Responses from the board are printed each time the loop
cycles, keeping the control flow easy to follow without extra threads.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Iterable, Optional

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from system_operations.arduino_serial_commuication.arduino_serial_commuication import (
        serialCommands,
    )
except Exception as exc:  # pragma: no cover - bail out if dependency missing
    print(f"Failed to import serialCommands: {exc}", file=sys.stderr)
    sys.exit(1)


def list_serial_ports() -> None:
    """Print a concise list of available serial devices."""

    try:
        import serial.tools.list_ports
    except Exception as exc:  # pragma: no cover - optional, best-effort only
        print(f"Unable to enumerate ports: {exc}")
        return

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports detected.")
        return

    for entry in ports:
        description = entry.description or "Unknown device"
        hwid = entry.hwid or ""
        suffix = f" ({hwid})" if hwid else ""
        print(f"{entry.device}: {description}{suffix}")


def drain_serial(link: serialCommands) -> None:
    """Read and display any queued lines from the Arduino."""

    try:
        lines = link.read_serial()
    except Exception as exc:
        print(f"Serial read failed: {exc}", file=sys.stderr)
        return

    if not lines:
        return

    for line in lines:
        print(f"[Arduino] {line}")


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Send newline-delimited commands to the Arduino over serial.",
    )
    parser.add_argument("--port", "-p", help="Serial port to open (auto-detects if omitted).")
    parser.add_argument(
        "--baud",
        "-b",
        type=int,
        default=115200,
        help="Baud rate for the connection (default: 115200).",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List detected serial ports and exit without opening anything.",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.list_ports:
        list_serial_ports()
        return 0

    try:
        link = serialCommands(port=args.port, baudrate=args.baud)
    except Exception as exc:
        print(f"Failed to open serial connection: {exc}", file=sys.stderr)
        return 1

    print("Connected to Arduino. Type commands; 'exit' or 'quit' closes the session.")

    exit_code = 0
    try:
        while True:
            drain_serial(link)
            try:
                raw = input("> ")
            except EOFError:
                print()
                break
            except KeyboardInterrupt:
                print("\nInterrupted by user.")
                break

            command = raw.strip()
            if not command:
                continue
            if command.lower() in {"exit", "quit"}:
                break

            try:
                link.send_serial(command)
            except Exception as exc:
                print(f"Failed to send command: {exc}", file=sys.stderr)
                exit_code = 1
                break

            time.sleep(0.05)  # Give the Arduino a moment to respond.
            drain_serial(link)
    finally:
        link.close_serial()

    print("Disconnected.")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
