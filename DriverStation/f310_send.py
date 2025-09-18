#!/usr/bin/env python3
import argparse
import fcntl
import glob
import os
import socket
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
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            name = read_device_name(fd)
        finally:
            os.close(fd)
        if target_name:
            if target_name.lower() in (name or '').lower():
                return path, name
        else:
            return path, name
    return None, None


def normalize_axis_value(v: int) -> float:
    if v <= -32767:
        return -1.0
    if v >= 32767:
        return 1.0
    return max(-1.0, min(1.0, v / 32767.0))


def quantize_axis(x: float, deadzone: float = 0.05) -> int:
    if abs(x) < deadzone:
        return 0
    x = max(-1.0, min(1.0, x))
    return int(round(x * 127.0))  # int8 range


def build_full_packet(seq: int, armed: bool, axes_i8: list[int], buttons_mask16: int) -> bytes:
    """
    12-byte full-state packet:
      [0]=0xA6 sync
      [1]=seq (u8)
      [2]=flags (bit0=armed)
      [3..8]=axes[0..5] as int8 (pad/truncate to 6)
      [9]=buttons low 8 bits
      [10]=buttons high 8 bits
      [11]=checksum XOR of bytes 0..10
    """
    flags = (1 if armed else 0) & 0xFF
    a = [(x & 0xFF) for x in (axes_i8 + [0] * 6)[:6]]
    b0 = buttons_mask16 & 0xFF
    b1 = (buttons_mask16 >> 8) & 0xFF
    header = bytes([0xA6, seq & 0xFF, flags] + a + [b0, b1])
    checksum = 0
    for b in header:
        checksum ^= b
    return header + bytes((checksum,))


def monitor_and_send(host: str, port: int, target_name: str | None, rate_hz: float,
                     hold_button: int, throttle_axis: int, steer_axis: int,
                     invert_throttle: bool, deadzone: float, idle_rate_hz: float) -> None:
    addr = (host, port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    fd = None
    axes: list[float] = []
    buttons: list[int] = []
    name = None
    seq = 0
    last_tx = 0.0

    min_period = 1.0 / max(1.0, rate_hz)
    idle_period = 1.0 / max(1.0, idle_rate_hz)

    print(f"Sending full state to {host}:{port} at {rate_hz:.1f} Hz (idle {idle_rate_hz:.1f} Hz)")

    try:
        while True:
            # Connect to joystick if needed
            if fd is None:
                path, name = find_matching_device(target_name)
                if not path:
                    # No joystick; send idle zero packets occasionally so robot stays stopped
                    now = time.monotonic()
                    if now - last_tx >= idle_period:
                        pkt = build_packet(seq, False, 0, 0)
                        try:
                            sock.sendto(pkt, addr)
                        except OSError:
                            pass
                        seq = (seq + 1) & 0xFF
                        last_tx = now
                    time.sleep(0.1)
                    continue

                try:
                    # Non-blocking so we can keep heartbeat even without new events
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                except OSError:
                    fd = None
                    time.sleep(0.2)
                    continue

                try:
                    num_axes = read_count(fd, JSIOCGAXES)
                    num_buttons = read_count(fd, JSIOCGBUTTONS)
                except OSError:
                    os.close(fd)
                    fd = None
                    time.sleep(0.2)
                    continue

                axes = [0.0] * num_axes
                buttons = [0] * num_buttons
                print(f"Controller connected: {name or 'Unknown'} ({num_axes} axes, {num_buttons} buttons)")

            # Drain all pending events (non-blocking)
            while True:
                try:
                    data = os.read(fd, 8)
                except BlockingIOError:
                    break
                except OSError:
                    try:
                        if fd is not None:
                            os.close(fd)
                    finally:
                        fd = None
                        print("Controller disconnected; sending idle keepalives.")
                        break

                if not data or len(data) < 8:
                    break
                try:
                    t_ms, value, etype, number = struct.unpack('IhBB', data)
                except struct.error:
                    continue

                effective_type = etype & ~JS_EVENT_INIT
                if effective_type == JS_EVENT_AXIS:
                    if 0 <= number < len(axes):
                        axes[number] = normalize_axis_value(value)
                elif effective_type == JS_EVENT_BUTTON:
                    if 0 <= number < len(buttons):
                        buttons[number] = 1 if value != 0 else 0

            # Periodic transmit regardless of new events
            now = time.monotonic()
            period = min_period
            armed = 0 <= hold_button < len(buttons) and buttons[hold_button] == 1

            if now - last_tx >= period:
                # Build full-state packet
                axes_i8 = [quantize_axis(a, deadzone) for a in axes[:6]]
                mask = 0
                for i in range(min(16, len(buttons))):
                    if buttons[i]:
                        mask |= (1 << i)
                pkt = build_full_packet(seq, armed, axes_i8, mask)
                try:
                    sock.sendto(pkt, addr)
                except OSError:
                    pass
                seq = (seq + 1) & 0xFF
                last_tx = now

    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(description="Send minimal joystick commands over UDP with deadman hold.")
    parser.add_argument("udp", nargs='?', default="127.0.0.1:9999",
                        help="Destination host:port (default 127.0.0.1:9999)")
    parser.add_argument("--name", default="Logitech Gamepad F310",
                        help="Substring of joystick name to match (empty for any)")
    parser.add_argument("--rate", type=float, default=50.0,
                        help="Max command rate in Hz (active)")
    parser.add_argument("--idle-rate", type=float, default=5.0,
                        help="Keepalive rate in Hz when no joystick")
    parser.add_argument("--hold-button", type=int, default=4,
                        help="Button index that must be held to move (deadman). Default 4=LB")
    parser.add_argument("--throttle-axis", type=int, default=1,
                        help="Axis index for forward/back (default 1 = left stick Y)")
    parser.add_argument("--invert-throttle", action="store_true", help="Invert throttle axis sign")
    parser.add_argument("--steer-axis", type=int, default=0,
                        help="Axis index for steering (default 0 = left stick X)")
    parser.add_argument("--deadzone", type=float, default=0.10,
                        help="Axis deadzone in [0..1]")
    args = parser.parse_args()

    if ':' not in args.udp:
        print("udp must be host:port", file=sys.stderr)
        sys.exit(2)
    host, port_s = args.udp.rsplit(':', 1)
    try:
        port = int(port_s)
    except ValueError:
        print("Invalid port", file=sys.stderr)
        sys.exit(2)

    target = args.name or None
    monitor_and_send(host, port, target, args.rate, args.hold_button,
                     args.throttle_axis, args.steer_axis, args.invert_throttle,
                     args.deadzone, args.idle_rate)


if __name__ == "__main__":
    main()
