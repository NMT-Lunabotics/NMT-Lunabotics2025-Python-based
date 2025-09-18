#!/usr/bin/env python3
import argparse
import fcntl
import glob
import os
import struct
import sys
import time
from array import array


# Linux joystick API constants (from linux/joystick.h)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def _IOC(dir_bits, type_chr, nr, size):
    return (dir_bits << 30) | (ord(type_chr) << 8) | (nr << 0) | (size << 16)


_IOC_READ = 2
JSIOCGAXES = _IOC(_IOC_READ, 'j', 0x11, 1)
JSIOCGBUTTONS = _IOC(_IOC_READ, 'j', 0x12, 1)


def JSIOCGNAME(length: int) -> int:
    return _IOC(_IOC_READ, 'j', 0x13, length)


def read_device_name(fd: int) -> str:
    buf = array('b', [0] * 128)
    try:
        fcntl.ioctl(fd, JSIOCGNAME(len(buf)), buf, True)
        raw = buf.tobytes().split(b"\x00", 1)[0]
        return raw.decode(errors="ignore") or ""
    except OSError:
        return ""


def read_count(fd: int, req: int) -> int:
    buf = array('B', [0])
    fcntl.ioctl(fd, req, buf, True)
    return int(buf[0])


def find_matching_device(target_name: str | None) -> tuple[str | None, str | None]:
    candidates = sorted(glob.glob('/dev/input/js*'))
    for path in candidates:
        try:
            # Open non-blocking just to read the name; close immediately.
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            name = read_device_name(fd)
        finally:
            os.close(fd)

        if target_name:
            # Case-insensitive contains match for flexibility
            if target_name.lower() in name.lower():
                return path, name
        else:
            # No target requested: take the first one we can read
            return path, name

    return None, None


def normalize_axis_value(v: int) -> float:
    # JS axis range is typically -32767..32767
    if v <= -32767:
        return -1.0
    if v >= 32767:
        return 1.0
    return max(-1.0, min(1.0, v / 32767.0))


def format_status(axes: list[float], buttons: list[int]) -> str:
    ax = ', '.join(f"{i}:{val:+.2f}" for i, val in enumerate(axes)) if axes else ""
    bt = ' '.join(f"{i}:{val}" for i, val in enumerate(buttons)) if buttons else ""
    return f"axes=[{ax}] buttons=[{bt}]"


def monitor_loop(target_name: str | None, refresh_idle: float) -> None:
    fd = None
    path = None
    name = None
    axes = []
    buttons = []
    last_print = 0.0

    try:
        while True:
            if fd is None:
                path, name = find_matching_device(target_name)
                if path is None:
                    # No device found; wait then retry
                    print("Waiting for gamepad... (plug in the controller)", end='\r', flush=True)
                    time.sleep(1.0)
                    continue

                try:
                    # Blocking open for event reads
                    fd = os.open(path, os.O_RDONLY)
                except OSError:
                    # Might have disappeared between scan and open; retry
                    fd = None
                    time.sleep(0.5)
                    continue

                try:
                    num_axes = read_count(fd, JSIOCGAXES)
                    num_buttons = read_count(fd, JSIOCGBUTTONS)
                except OSError:
                    os.close(fd)
                    fd = None
                    time.sleep(0.5)
                    continue

                axes = [0.0] * num_axes
                buttons = [0] * num_buttons
                sys.stdout.write("\n")
                print(f"Connected: {name or 'Unknown'} at {path}")

                # On connect, print an initial status
                print(format_status(axes, buttons))

            try:
                data = os.read(fd, 8)
                if len(data) < 8:
                    # Should not happen on blocking reads; treat as transient
                    time.sleep(0.005)
                    continue
            except OSError:
                # Device likely disconnected
                try:
                    if fd is not None:
                        os.close(fd)
                finally:
                    fd = None
                    print("\nController disconnected. Waiting for reconnection...")
                    time.sleep(0.5)
                    continue

            try:
                t_ms, value, etype, number = struct.unpack('IhBB', data)
            except struct.error:
                # Malformed; skip
                continue

            effective_type = etype & ~JS_EVENT_INIT
            if effective_type == JS_EVENT_AXIS:
                if 0 <= number < len(axes):
                    axes[number] = normalize_axis_value(value)
            elif effective_type == JS_EVENT_BUTTON:
                if 0 <= number < len(buttons):
                    buttons[number] = 1 if value != 0 else 0

            now = time.time()
            if now - last_print >= refresh_idle:
                # Print a single-line status updated in place
                sys.stdout.write("\r" + format_status(axes, buttons) + " " * 8)
                sys.stdout.flush()
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except Exception:
                pass
        sys.stdout.write("\nExiting.\n")


def main():
    parser = argparse.ArgumentParser(description="Display Logitech F310 (or any Linux joystick) inputs.")
    parser.add_argument(
        "--name",
        default="Logitech Gamepad F310",
        help="Substring of joystick name to match (case-insensitive). Use empty to match any.",
    )
    parser.add_argument(
        "--any",
        action="store_true",
        help="Ignore name matching and attach to the first joystick found.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=0.02,
        help="Minimum seconds between status prints (default: 0.02 ~50Hz).",
    )
    args = parser.parse_args()

    target = None if args.any else (args.name or None)
    monitor_loop(target, refresh_idle=max(0.001, args.rate))


if __name__ == "__main__":
    main()

