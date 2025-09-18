#!/usr/bin/env python3
import argparse
import fcntl
import glob
import os
import queue
import socket
import struct
import threading
import time
from array import array
from dataclasses import dataclass

import tkinter as tk
from tkinter import ttk
try:
    from PIL import Image, ImageTk  # type: ignore
except Exception:  # Pillow optional
    Image = None
    ImageTk = None


# Linux joystick API constants
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
    flags = (1 if armed else 0) & 0xFF
    a = [(x & 0xFF) for x in (axes_i8 + [0] * 6)[:6]]
    b0 = buttons_mask16 & 0xFF
    b1 = (buttons_mask16 >> 8) & 0xFF
    header = bytes([0xA6, seq & 0xFF, flags] + a + [b0, b1])
    checksum = 0
    for b in header:
        checksum ^= b
    return header + bytes((checksum,))


def build_text_message(seq: int, text: str) -> bytes:
    payload = text.encode('utf-8')[:220]
    header = bytes((0xB0, seq & 0xFF, len(payload))) + payload
    csum = 0
    for b in header:
        csum ^= b
    return header + bytes((csum,))


@dataclass
class JoyState:
    axes: list
    buttons: list
    name: str = ""


class JoystickReader(threading.Thread):
    def __init__(self, target_name: str | None, deadzone: float = 0.1):
        super().__init__(daemon=True)
        self.target_name = target_name
        self.deadzone = deadzone
        self.running = True
        self.state = JoyState([0.0] * 6, [0] * 16, "")
        self._fd = None

    def stop(self):
        self.running = False
        try:
            if self._fd is not None:
                os.close(self._fd)
        except Exception:
            pass

    def run(self):
        while self.running:
            if self._fd is None:
                path, name = find_matching_device(self.target_name)
                if not path:
                    time.sleep(0.5)
                    continue
                try:
                    self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                except OSError:
                    self._fd = None
                    time.sleep(0.5)
                    continue
                try:
                    na = read_count(self._fd, JSIOCGAXES)
                    nb = read_count(self._fd, JSIOCGBUTTONS)
                except OSError:
                    os.close(self._fd)
                    self._fd = None
                    time.sleep(0.5)
                    continue
                self.state = JoyState([0.0] * max(6, na), [0] * max(16, nb), name or "")

            # Drain events
            try:
                data = os.read(self._fd, 8)
            except BlockingIOError:
                data = None
            except OSError:
                try:
                    os.close(self._fd)
                except Exception:
                    pass
                self._fd = None
                continue

            if not data:
                time.sleep(0.01)
                continue
            if len(data) < 8:
                continue
            try:
                _, value, etype, number = struct.unpack('IhBB', data)
            except struct.error:
                continue
            effective = etype & ~JS_EVENT_INIT
            if effective == JS_EVENT_AXIS:
                if 0 <= number < len(self.state.axes):
                    self.state.axes[number] = normalize_axis_value(value)
            elif effective == JS_EVENT_BUTTON:
                if 0 <= number < len(self.state.buttons):
                    self.state.buttons[number] = 1 if value != 0 else 0


class UDPSession(threading.Thread):
    def __init__(self, host: str, port: int, joy: JoystickReader,
                 hold_button: int = 4, rate_hz: float = 50.0, deadzone: float = 0.1):
        super().__init__(daemon=True)
        self.addr = (host, port)
        self.joy = joy
        self.hold_button = hold_button
        self.period = 1.0 / max(1.0, rate_hz)
        self.deadzone = deadzone
        self.running = True
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        # Bind to receive messages back
        self.sock.bind(("0.0.0.0", 0))
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass

    def _rx_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                time.sleep(0.01)
                continue
            if not data:
                continue
            # Accept plain text messages (0xB0) and log others briefly
            if data[0] == 0xB0 and len(data) >= 4:
                # Validate checksum
                csum = 0
                for b in data[:-1]:
                    csum ^= b
                if csum == data[-1]:
                    ln = data[2]
                    payload = data[3:3 + ln]
                    try:
                        text = payload.decode('utf-8', errors='replace')
                    except Exception:
                        text = str(payload)
                    self.rx_queue.put(text)
            else:
                self.rx_queue.put(f"[rx {len(data)} bytes]")

    def send_text(self, text: str):
        pkt = build_text_message(self.seq, text)
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError:
            pass
        self.seq = (self.seq + 1) & 0xFF

    def run(self):
        self._rx_thread.start()
        last = 0.0
        while self.running:
            now = time.monotonic()
            if now - last >= self.period:
                axes = self.joy.state.axes
                buttons = self.joy.state.buttons
                axes_i8 = [quantize_axis(a, self.deadzone) for a in axes[:6]]
                mask = 0
                for i in range(min(16, len(buttons))):
                    if buttons[i]:
                        mask |= (1 << i)
                armed = (0 <= self.hold_button < len(buttons) and buttons[self.hold_button] == 1)
                pkt = build_full_packet(self.seq, armed, axes_i8, mask)
                try:
                    self.sock.sendto(pkt, self.addr)
                except OSError:
                    pass
                self.seq = (self.seq + 1) & 0xFF
                last = now
            time.sleep(0.005)


class MapPanel(ttk.Frame):
    def __init__(self, parent, image_path: str | None = None):
        super().__init__(parent)
        self.canvas = tk.Canvas(self, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.image_path = image_path
        self._img_pil = None
        self._photo = None
        self._img_item = None
        self.bind("<Configure>", lambda e: self._draw())
        self._load()

    def _load(self):
        self._img_pil = None
        self._photo = None
        if self.image_path and os.path.exists(self.image_path):
            if Image is not None:
                try:
                    self._img_pil = Image.open(self.image_path)
                except Exception:
                    self._img_pil = None
            else:
                # Fallback: try Tk PhotoImage directly (may support PNG)
                try:
                    self._photo = tk.PhotoImage(file=self.image_path)
                except Exception:
                    self._photo = None
        self._draw()

    def _draw(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width() or 10
        h = c.winfo_height() or 10
        if self._img_pil is not None and ImageTk is not None:
            # Fit image to canvas while preserving aspect ratio
            iw, ih = self._img_pil.size
            if iw <= 0 or ih <= 0:
                iw, ih = 1, 1
            scale = min(w / iw, h / ih)
            tw, th = max(1, int(iw * scale)), max(1, int(ih * scale))
            img = self._img_pil.resize((tw, th))
            self._photo = ImageTk.PhotoImage(img)
            self._img_item = c.create_image(w // 2, h // 2, image=self._photo)
        elif self._photo is not None:
            # Unscaled center
            self._img_item = c.create_image(w // 2, h // 2, image=self._photo)
        else:
            # Placeholder box with instructions
            c.create_rectangle(10, 10, w - 10, h - 10, outline="#888")
            msg = "No map image found. Place file and restart."
            c.create_text(w // 2, h // 2, text=msg)


class Dashboard(tk.Tk):
    def __init__(self, host: str, port: int, target_name: str | None):
        super().__init__()
        self.title("F310 Robot Control GUI")
        self.geometry("1200x800")

        self.joy = JoystickReader(target_name)
        self.udp = UDPSession(host, port, self.joy)

        # Layout: left/right split
        self.main_split = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        self.main_split.pack(fill=tk.BOTH, expand=True)

        self.left_frame = ttk.Frame(self.main_split)
        self.right_frame = ttk.Frame(self.main_split)
        self.main_split.add(self.left_frame, weight=1)
        self.main_split.add(self.right_frame, weight=1)

        # Right split: top controls (75%), bottom terminal (25%)
        self.right_split = ttk.Panedwindow(self.right_frame, orient=tk.VERTICAL)
        self.right_split.pack(fill=tk.BOTH, expand=True)

        self.controls_frame = ttk.Frame(self.right_split, padding=10)
        self.terminal_frame = ttk.Frame(self.right_split)
        self.right_split.add(self.controls_frame, weight=3)
        self.right_split.add(self.terminal_frame, weight=1)

        # Left pane: Camera/Map tabs (placeholder)
        self.tabs = ttk.Notebook(self.left_frame)
        self.tab_camera = ttk.Frame(self.tabs)
        self.tab_map = ttk.Frame(self.tabs)
        self.tabs.add(self.tab_camera, text="Camera")
        self.tabs.add(self.tab_map, text="Map")
        self.tabs.pack(fill=tk.BOTH, expand=True)
        ttk.Label(self.tab_camera, text="Camera feed placeholder").pack(pady=20)
        # Map placeholder image panel
        self.map_panel = MapPanel(self.tab_map, getattr(self, "map_image_path", None))
        self.map_panel.pack(fill=tk.BOTH, expand=True)

        # Controls & indicators
        row = 0
        self.lbl_conn = ttk.Label(self.controls_frame, text="Controller: disconnected")
        self.lbl_conn.grid(row=row, column=0, columnspan=3, sticky="w")
        row += 1

        self._add_progress(row, "Actuator", 0)
        row += 1
        self._add_progress(row, "Speed", 0)
        row += 1
        self._add_progress(row, "Battery", 0)
        row += 1
        self._add_progress(row, "Current", 0)
        row += 1

        # Compass
        ttk.Label(self.controls_frame, text="Compass").grid(row=row, column=0, sticky="w")
        self.canvas_compass = tk.Canvas(self.controls_frame, width=120, height=120, bg="white")
        self.canvas_compass.grid(row=row, column=1, sticky="w")
        self._draw_compass(heading_deg=0.0)
        row += 1

        # Axes/buttons readout
        self.lbl_axes = ttk.Label(self.controls_frame, text="Axes: []")
        self.lbl_axes.grid(row=row, column=0, columnspan=3, sticky="w")
        row += 1
        self.lbl_buttons = ttk.Label(self.controls_frame, text="Buttons: []")
        self.lbl_buttons.grid(row=row, column=0, columnspan=3, sticky="w")
        row += 1

        for i in range(3):
            self.controls_frame.grid_columnconfigure(i, weight=1)

        # Terminal/chat UI
        self.txt_log = tk.Text(self.terminal_frame, height=8, wrap=tk.WORD, state=tk.DISABLED)
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=8, pady=(8, 4))
        entry_row = ttk.Frame(self.terminal_frame)
        entry_row.pack(fill=tk.X, padx=8, pady=(0, 8))
        self.entry_msg = ttk.Entry(entry_row)
        self.entry_msg.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.btn_send = ttk.Button(entry_row, text="Send", command=self._on_send)
        self.btn_send.pack(side=tk.LEFT, padx=6)

        # Start threads
        self.joy.start()
        self.udp.start()

        # Drive periodic UI updates
        self.after(100, self._update_ui)

        # Set initial pane ratios after window draws
        self.after(200, self._set_initial_sashes)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _add_progress(self, row: int, label: str, init_value: float):
        ttk.Label(self.controls_frame, text=label).grid(row=row, column=0, sticky="w")
        pb = ttk.Progressbar(self.controls_frame, orient=tk.HORIZONTAL, mode='determinate', length=200)
        pb.grid(row=row, column=1, sticky="we")
        pb['maximum'] = 100
        pb['value'] = int(init_value * 100)
        setattr(self, f"pb_{label.lower()}", pb)
        val_lbl = ttk.Label(self.controls_frame, text=f"{init_value:.2f}")
        val_lbl.grid(row=row, column=2, sticky="e")
        setattr(self, f"lbl_{label.lower()}_val", val_lbl)

    def _draw_compass(self, heading_deg: float):
        c = self.canvas_compass
        c.delete("all")
        w = int(c['width']); h = int(c['height'])
        cx, cy = w // 2, h // 2
        r = min(cx, cy) - 5
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline="black")
        # Heading line (north-up circle, 0 deg points up)
        import math
        ang = math.radians(90 - heading_deg)
        x = cx + r * math.cos(ang)
        y = cy - r * math.sin(ang)
        c.create_line(cx, cy, x, y, fill="red", width=3)
        c.create_text(cx, cy + r + 10, text=f"{heading_deg:.0f}°")

    def _append_log(self, text: str):
        self.txt_log.configure(state=tk.NORMAL)
        self.txt_log.insert(tk.END, text + "\n")
        self.txt_log.see(tk.END)
        self.txt_log.configure(state=tk.DISABLED)

    def _on_send(self):
        msg = self.entry_msg.get().strip()
        if not msg:
            return
        self.udp.send_text(msg)
        self._append_log(f"> {msg}")
        self.entry_msg.delete(0, tk.END)

    def _set_initial_sashes(self):
        # Approximate 50/50 split horizontally
        try:
            total_w = self.main_split.winfo_width()
            self.main_split.sashpos(0, total_w // 2)
        except Exception:
            pass
        # Right vertical split: 75% controls, 25% terminal
        try:
            total_h = self.right_split.winfo_height()
            self.right_split.sashpos(0, int(total_h * 0.75))
        except Exception:
            pass

    def _update_ui(self):
        # Update connection label and axes/buttons
        name = self.joy.state.name or "disconnected"
        self.lbl_conn.configure(text=f"Controller: {name}")

        axes = self.joy.state.axes[:6]
        buttons = self.joy.state.buttons[:16]
        axes_str = ", ".join(f"{i}:{v:+.2f}" for i, v in enumerate(axes))
        self.lbl_axes.configure(text=f"Axes: [{axes_str}]")
        pressed = [str(i) for i, b in enumerate(buttons) if b]
        self.lbl_buttons.configure(text=f"Buttons: [{', '.join(pressed)}]")

        # Example: map axes to speed/actuator for display only
        thr = axes[1] if len(axes) > 1 else 0.0
        steer = axes[0] if len(axes) > 0 else 0.0
        spd_val = abs(thr)
        self.pb_speed['value'] = int(spd_val * 100)
        self.lbl_speed_val.configure(text=f"{spd_val:.2f}")

        # Battery/Current placeholders (update via incoming messages if available)
        # Keep existing values; nothing here changes them.

        # Drain any received messages
        while True:
            try:
                m = self.udp.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._append_log(m)
            # Optional: parse telemetry like "battery=12.1 current=0.8 heading=15"
            parts = dict(p.split('=') for p in m.split() if '=' in p)
            if 'battery' in parts:
                try:
                    b = float(parts['battery'])
                    val = max(0.0, min(1.0, (b - 10.0) / 2.6))  # crude 10-12.6V scale
                    self.pb_battery['value'] = int(val * 100)
                    self.lbl_battery_val.configure(text=f"{b:.2f}V")
                except Exception:
                    pass
            if 'current' in parts:
                try:
                    a = float(parts['current'])
                    val = max(0.0, min(1.0, a / 20.0))
                    self.pb_current['value'] = int(val * 100)
                    self.lbl_current_val.configure(text=f"{a:.2f}A")
                except Exception:
                    pass
            if 'act' in parts:
                try:
                    act = float(parts['act'])
                    val = max(0.0, min(1.0, act))
                    self.pb_actuator['value'] = int(val * 100)
                    self.lbl_actuator_val.configure(text=f"{act:.2f}")
                except Exception:
                    pass
            if 'heading' in parts:
                try:
                    deg = float(parts['heading'])
                    self._draw_compass(deg)
                except Exception:
                    pass

        self.after(50, self._update_ui)

    def _on_close(self):
        self.udp.stop()
        self.joy.stop()
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description="GUI sender for robot control with chat and indicators.")
    parser.add_argument("udp", nargs='?', default="127.0.0.1:9999",
                        help="Destination host:port (default 127.0.0.1:9999)")
    parser.add_argument("--name", default="Logitech Gamepad F310",
                        help="Joystick name substring (empty for any)")
    parser.add_argument("--map-image", default="map_placeholder.png",
                        help="Path to map placeholder image (PNG/JPG)")
    args = parser.parse_args()

    if ':' not in args.udp:
        host, port = '127.0.0.1', 9999
    else:
        host, port_s = args.udp.rsplit(':', 1)
        try:
            port = int(port_s)
        except ValueError:
            host, port = '127.0.0.1', 9999

    # Resolve map image path robustly: prefer CLI path, else script-dir fallback
    def resolve_map_path(p: str) -> str | None:
        if not p:
            return None
        if os.path.isabs(p) and os.path.exists(p):
            return p
        # try CWD
        if os.path.exists(p):
            return os.path.abspath(p)
        # try script directory
        here = os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.join(here, p)
        if os.path.exists(candidate):
            return candidate
        return None

    target = args.name or None
    app = Dashboard(host, port, target)
    map_path = resolve_map_path(args.map_image)
    if map_path is not None:
        try:
            app.map_panel.image_path = map_path
            app.map_panel._load()
            print(f"Loaded map image: {map_path}")
        except Exception as e:
            print(f"Could not load map image '{map_path}': {e}")
    else:
        print(f"Map image not found: {args.map_image}")
    app.mainloop()


if __name__ == "__main__":
    main()
