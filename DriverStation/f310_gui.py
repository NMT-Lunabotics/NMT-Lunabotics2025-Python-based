#!/usr/bin/env python3
"""Simple Tk-based driver station for the Logitech F310 controller.

This GUI mirrors the functionality of ``f310_send.py`` while providing a
visual overview of the controller state, telemetry returned from the robot,
and a bottom-right command console for sending ad-hoc text commands.  The
script intentionally avoids command-line arguments; tweak the constants near
the top to adjust destinations or ports.

Telemetry reception expects JSON objects (bytes or text) delivered over UDP to
``TELEMETRY_LISTEN``.  For example::

    echo '{"voltage": 12.4, "current": 3.8, "temperature": 32}' \
        | socat - UDP-DATAGRAM:127.0.0.1:10000

Command console submissions are transmitted verbatim (UTF-8 encoded) to
``COMMAND_DESTINATION`` over UDP.
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time
from array import array
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError as exc:  # pragma: no cover - tkinter unavailable on some envs
    raise SystemExit(f"tkinter is required for this GUI: {exc}")

# --- Controller/UDP configuration -------------------------------------------------

TARGET_CONTROLLER_NAME = "Logitech Gamepad F310"
UDP_DESTINATION: Tuple[str, int] = ("127.0.0.1", 11000)
SEND_RATE_HZ = 50.0
IDLE_RATE_HZ = 50.0  # keep the outbound stream at a constant 50 Hz
DEADZONE = 0.10
HOLD_BUTTON_INDEX = 4  # LB acts as deadman switch by default
MAX_DISPLAY_AXES = 6
MAX_DISPLAY_BUTTONS = 12
TELEMETRY_LISTEN: Tuple[str, int] = ("0.0.0.0", 10000)
COMMAND_DESTINATION: Tuple[str, int] = ("127.0.0.1", 10001)

# --- UI colors -------------------------------------------------------------------

WINDOW_BG = "#141821"
PANEL_BG = "#1f2430"
PANEL_INNER_BG = "#242a38"
CANVAS_BG = "#10141f"
BORDER_COLOR = "#31384a"
GRID_COLOR = "#3f475c"
ACCENT_COLOR = "#4a90e2"
ACCENT_ACTIVE = "#5aa0ff"
TEXT_PRIMARY = "#f4f7ff"
TEXT_MUTED = "#9aa3c2"

# --- Linux joystick constants -----------------------------------------------------

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def _IOC(dir_bits: int, type_chr: str, nr: int, size: int) -> int:
    return (dir_bits << 30) | (ord(type_chr) << 8) | (nr << 0) | (size << 16)


_IOC_READ = 2
JSIOCGAXES = _IOC(_IOC_READ, "j", 0x11, 1)
JSIOCGBUTTONS = _IOC(_IOC_READ, "j", 0x12, 1)


def JSIOCGNAME(length: int) -> int:
    return _IOC(_IOC_READ, "j", 0x13, length)


# --- Helper utilities -------------------------------------------------------------

def read_device_name(fd: int) -> str:
    buf = array("b", [0] * 128)
    try:
        import fcntl

        fcntl.ioctl(fd, JSIOCGNAME(len(buf)), buf, True)
        raw = buf.tobytes().split(b"\x00", 1)[0]
        return raw.decode(errors="ignore") or ""
    except OSError:
        return ""


def read_count(fd: int, req: int) -> int:
    import fcntl

    buf = array("B", [0])
    fcntl.ioctl(fd, req, buf, True)
    return int(buf[0])


def find_matching_device(target_name: Optional[str]) -> Tuple[Optional[str], Optional[str]]:
    import glob

    candidates = sorted(glob.glob("/dev/input/js*"))
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
            if target_name.lower() in (name or "").lower():
                return path, name
        else:
            return path, name
    return None, None


def normalize_axis_value(raw: int) -> float:
    if raw <= -32767:
        return -1.0
    if raw >= 32767:
        return 1.0
    return max(-1.0, min(1.0, raw / 32767.0))


def quantize_axis(value: float, deadzone: float) -> int:
    if abs(value) < deadzone:
        return 0
    value = max(-1.0, min(1.0, value))
    return int(round(value * 127.0))


def build_full_packet(seq: int, armed: bool, axes_i8: List[int], buttons_mask16: int) -> bytes:
    flags = 1 if armed else 0
    a = [(x & 0xFF) for x in (axes_i8 + [0] * MAX_DISPLAY_AXES)[:MAX_DISPLAY_AXES]]
    low = buttons_mask16 & 0xFF
    high = (buttons_mask16 >> 8) & 0xFF
    header = bytes([0xA6, seq & 0xFF, flags] + a + [low, high])
    checksum = 0
    for b in header:
        checksum ^= b
    return header + bytes((checksum,))


# --- Data containers --------------------------------------------------------------

@dataclass
class ControllerSnapshot:
    axes: List[float]
    buttons: List[int]
    connected: bool
    name: str


@dataclass
class TelemetrySnapshot:
    voltage: Optional[float]
    current: Optional[float]
    temperature: Optional[float]
    last_update: Optional[float]


class JoystickWorker:
    """Background reader that keeps track of the current joystick state."""

    def __init__(self, target_name: Optional[str] = TARGET_CONTROLLER_NAME) -> None:
        self._target_name = target_name
        self._lock = threading.Lock()
        self._axes: List[float] = []
        self._buttons: List[int] = []
        self._connected = False
        self._name = ""
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="JoystickWorker", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def join(self, timeout: Optional[float] = None) -> None:
        self._thread.join(timeout)

    def snapshot(self) -> ControllerSnapshot:
        with self._lock:
            return ControllerSnapshot(
                axes=list(self._axes),
                buttons=list(self._buttons),
                connected=self._connected,
                name=self._name,
            )

    # Internal ------------------------------------------------------------------

    def _run(self) -> None:
        fd: Optional[int] = None
        while not self._stop_event.is_set():
            if fd is None:
                path, name = find_matching_device(self._target_name)
                if not path:
                    self._mark_disconnected()
                    self._stop_event.wait(0.4)
                    continue
                try:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                except OSError:
                    fd = None
                    self._stop_event.wait(0.3)
                    continue
                try:
                    axis_count = read_count(fd, JSIOCGAXES)
                    button_count = read_count(fd, JSIOCGBUTTONS)
                except OSError:
                    os.close(fd)
                    fd = None
                    self._stop_event.wait(0.3)
                    continue
                with self._lock:
                    self._axes = [0.0] * axis_count
                    self._buttons = [0] * button_count
                    self._connected = True
                    self._name = name or path
                continue

            try:
                data = os.read(fd, 8)
            except BlockingIOError:
                self._stop_event.wait(0.01)
                continue
            except OSError:
                os.close(fd)
                fd = None
                self._mark_disconnected()
                continue

            if not data or len(data) < 8:
                self._stop_event.wait(0.01)
                continue

            try:
                _, value, etype, number = struct.unpack("IhBB", data)
            except struct.error:
                continue

            effective_type = etype & ~JS_EVENT_INIT
            with self._lock:
                if effective_type == JS_EVENT_AXIS:
                    if 0 <= number < len(self._axes):
                        self._axes[number] = normalize_axis_value(value)
                elif effective_type == JS_EVENT_BUTTON:
                    if 0 <= number < len(self._buttons):
                        self._buttons[number] = 1 if value else 0

        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _mark_disconnected(self) -> None:
        with self._lock:
            self._connected = False
            self._name = ""
            if self._axes:
                self._axes = [0.0] * len(self._axes)
            if self._buttons:
                self._buttons = [0] * len(self._buttons)


class TelemetryListener:
    """Listens for UDP telemetry updates and exposes the latest sample."""

    def __init__(self, listen_addr: Tuple[str, int] = TELEMETRY_LISTEN) -> None:
        self._lock = threading.Lock()
        self._state: Dict[str, Optional[float]] = {"voltage": None, "current": None, "temperature": None}
        self._last_update: Optional[float] = None
        self._stop_event = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._thread = threading.Thread(target=self._run, name="TelemetryListener", daemon=True)
        self._listen_addr = listen_addr

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock:
            self._sock.close()

    def join(self, timeout: Optional[float] = None) -> None:
        self._thread.join(timeout)

    def snapshot(self) -> TelemetrySnapshot:
        with self._lock:
            return TelemetrySnapshot(
                voltage=self._state.get("voltage"),
                current=self._state.get("current"),
                temperature=self._state.get("temperature"),
                last_update=self._last_update,
            )

    # Internal ------------------------------------------------------------------

    def _run(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(self._listen_addr)
            sock.settimeout(0.5)
            self._sock = sock
        except OSError as exc:
            print(f"Telemetry listener disabled: {exc}")
            return

        while not self._stop_event.is_set():
            try:
                data, _ = sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break

            payload = data.decode(errors="ignore").strip()
            if not payload:
                continue

            parsed = self._parse_payload(payload)
            if not parsed:
                continue

            with self._lock:
                self._state.update(parsed)
                self._last_update = time.time()

        sock.close()

    @staticmethod
    def _parse_payload(payload: str) -> Dict[str, Optional[float]]:
        try:
            data = json.loads(payload)
            if isinstance(data, dict):
                return {
                    "voltage": TelemetryListener._coerce_float(data.get("voltage")),
                    "current": TelemetryListener._coerce_float(data.get("current")),
                    "temperature": TelemetryListener._coerce_float(data.get("temperature")),
                }
        except json.JSONDecodeError:
            pass

        result: Dict[str, Optional[float]] = {}
        for token in payload.replace(",", " ").split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            if key in {"voltage", "current", "temperature"}:
                result[key] = TelemetryListener._coerce_float(value)
        return result

    @staticmethod
    def _coerce_float(value: object) -> Optional[float]:
        try:
            if value is None:
                return None
            return float(value)
        except (TypeError, ValueError):
            return None


# --- GUI -------------------------------------------------------------------------

class CommandConsole(ttk.LabelFrame):
    """Simple command prompt area for sending manual robot commands."""

    def __init__(self, parent: tk.Widget, sender: Callable[[str], Optional[str]]) -> None:
        super().__init__(parent, text="Robot Console", padding=10, style="Panel.TLabelframe")
        self._sender = sender

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self._output = tk.Text(
            self,
            height=8,
            state="disabled",
            wrap="word",
            background=CANVAS_BG,
            foreground=TEXT_PRIMARY,
            insertbackground=TEXT_PRIMARY,
            relief="flat",
            borderwidth=1,
            highlightthickness=1,
            highlightbackground=BORDER_COLOR,
        )
        self._output.grid(row=0, column=0, columnspan=2, sticky="nsew")

        self._entry_var = tk.StringVar()
        self._entry = ttk.Entry(self, textvariable=self._entry_var, style="Console.TEntry")
        self._entry.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        self._entry.bind("<Return>", self._on_submit)

        send_btn = ttk.Button(self, text="Send", command=self._on_submit, style="Accent.TButton")
        send_btn.grid(row=1, column=1, padx=(6, 0), pady=(6, 0))

    def focus_entry(self) -> None:
        self._entry.focus_set()

    def append(self, line: str) -> None:
        self._output.configure(state="normal")
        timestamp = time.strftime("%H:%M:%S")
        self._output.insert("end", f"[{timestamp}] {line}\n")
        self._output.see("end")
        self._output.configure(state="disabled")

    def _on_submit(self, *_args) -> None:
        raw = self._entry_var.get().strip()
        if not raw:
            return
        self._entry_var.set("")
        self.append(f"> {raw}")
        try:
            error = self._sender(raw)
        except Exception as exc:  # pragma: no cover - defensive
            self.append(f"! send failed: {exc}")
        else:
            if error:
                self.append(f"! {error}")


class ControlStationGUI:
    def __init__(
        self,
        root: tk.Tk,
        joystick: JoystickWorker,
        telemetry: TelemetryListener,
    ) -> None:
        self.root = root
        self.joystick = joystick
        self.telemetry = telemetry

        self._style = ttk.Style()
        self._configure_style()

        self.root.title("F310 Driver Station")
        self.root.geometry("1000x600")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._setup_layout()

        self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_socket.setblocking(False)
        self._seq = 0
        self._last_tx = 0.0

        self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._command_socket.setblocking(False)

        self._schedule_update()

    # Layout --------------------------------------------------------------------

    def _setup_layout(self) -> None:
        main = ttk.Frame(self.root, padding=12)
        main.configure(style="Main.TFrame")
        main.pack(fill="both", expand=True)

        main.grid_columnconfigure(0, weight=3)
        main.grid_columnconfigure(1, weight=2)
        main.grid_rowconfigure(0, weight=3)
        main.grid_rowconfigure(1, weight=2)

        self.view_frame = ttk.LabelFrame(main, text="Visualization", padding=10, style="Panel.TLabelframe")
        self.view_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 12), pady=(0, 12))

        self.view_notebook = ttk.Notebook(self.view_frame)
        self.view_notebook.pack(fill="both", expand=True)

        self.map_tab = ttk.Frame(self.view_notebook, style="PanelInner.TFrame")
        self.camera_tab = ttk.Frame(self.view_notebook, style="PanelInner.TFrame")
        self.view_notebook.add(self.map_tab, text="Map")
        self.view_notebook.add(self.camera_tab, text="Camera")

        self.map_placeholder = ttk.Label(
            self.map_tab,
            text="Map view placeholder",
            anchor="center",
            style="Placeholder.TLabel",
        )
        self.map_placeholder.pack(fill="both", expand=True, padx=4, pady=4)

        self.camera_placeholder = ttk.Label(
            self.camera_tab,
            text="Camera feed placeholder",
            anchor="center",
            style="Placeholder.TLabel",
        )
        self.camera_placeholder.pack(fill="both", expand=True, padx=4, pady=4)

        self.telemetry_frame = ttk.LabelFrame(main, text="Robot Feedback", padding=10, style="Panel.TLabelframe")
        self.telemetry_frame.grid(row=0, column=1, sticky="nsew", padx=(12, 0), pady=(0, 12))

        stats_container = ttk.Frame(self.telemetry_frame, style="PanelInner.TFrame")
        stats_container.pack(fill="x")
        self.voltage_var = tk.StringVar(value="--")
        self.current_var = tk.StringVar(value="--")
        self.temperature_var = tk.StringVar(value="--")
        self.last_update_var = tk.StringVar(value="No data")

        self._add_stat_row(stats_container, "Voltage", self.voltage_var, "V")
        self._add_stat_row(stats_container, "Current", self.current_var, "A")
        self._add_stat_row(stats_container, "Temperature", self.temperature_var, "°C")

        ttk.Label(stats_container, textvariable=self.last_update_var, style="Muted.TLabel").grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(6, 0)
        )

        self.controller_frame = ttk.Frame(main, padding=10, style="Panel.TFrame")
        self.controller_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 12), pady=(12, 0))

        self.command_console = CommandConsole(main, self._send_console_command)
        self.command_console.grid(row=1, column=1, sticky="nsew", padx=(12, 0), pady=(12, 0))
        self.command_console.focus_entry()

        self.device_label_var = tk.StringVar(value="Controller: searching...")
        ttk.Label(
            self.controller_frame,
            textvariable=self.device_label_var,
            style="Header.TLabel",
        ).pack(anchor="w", pady=(0, 4))
        self.controller_summary_var = tk.StringVar(
            value="Controller indicators are disabled."
        )
        ttk.Label(
            self.controller_frame,
            textvariable=self.controller_summary_var,
            style="Muted.TLabel",
            justify="left",
            anchor="nw",
        ).pack(fill="both", expand=True, pady=(6, 4))

    def _add_stat_row(self, parent: ttk.Frame, label: str, value_var: tk.StringVar, unit: str) -> None:
        row = parent.grid_size()[1]
        ttk.Label(parent, text=f"{label}:", style="Subheading.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        ttk.Label(parent, textvariable=value_var, style="Header.TLabel").grid(row=row, column=1, sticky="w", pady=4)
        ttk.Label(parent, text=unit, style="Muted.TLabel").grid(row=row, column=2, sticky="w", padx=(4, 0))

    # Update loop ----------------------------------------------------------------

    def _configure_style(self) -> None:
        try:
            if "clam" in self._style.theme_names():
                self._style.theme_use("clam")
        except tk.TclError:
            pass

        self.root.configure(background=WINDOW_BG)

        self._style.configure("Main.TFrame", background=WINDOW_BG)
        self._style.configure("Panel.TLabelframe", background=PANEL_BG, foreground=TEXT_PRIMARY, borderwidth=0, relief="flat")
        self._style.configure("Panel.TLabelframe.Label", background=PANEL_BG, foreground=TEXT_PRIMARY, font=("TkDefaultFont", 12, "bold"))
        self._style.configure("Panel.TFrame", background=PANEL_BG)
        self._style.configure("PanelInner.TFrame", background=PANEL_BG)

        self._style.configure("TLabel", background=PANEL_BG, foreground=TEXT_PRIMARY)
        self._style.configure("Header.TLabel", background=PANEL_BG, foreground=TEXT_PRIMARY, font=("TkDefaultFont", 11, "bold"))
        self._style.configure("Subheading.TLabel", background=PANEL_BG, foreground=ACCENT_COLOR, font=("TkDefaultFont", 10, "bold"))
        self._style.configure("Muted.TLabel", background=PANEL_BG, foreground=TEXT_MUTED)
        self._style.configure("Placeholder.TLabel", background=CANVAS_BG, foreground=TEXT_MUTED, padding=20, anchor="center")

        self._style.configure("Joystick.TCheckbutton", background=PANEL_BG, foreground=TEXT_MUTED)
        self._style.map(
            "Joystick.TCheckbutton",
            foreground=[("selected", ACCENT_COLOR), ("!disabled", TEXT_MUTED)],
        )

        self._style.configure("Accent.TButton", background=ACCENT_COLOR, foreground="#ffffff", borderwidth=0, padding=(14, 8))
        self._style.map(
            "Accent.TButton",
            background=[("pressed", ACCENT_ACTIVE), ("active", ACCENT_ACTIVE), ("disabled", BORDER_COLOR)],
            foreground=[("disabled", TEXT_MUTED)],
        )

        entry_field_bg = "#1a1f2d"
        self._style.configure(
            "Console.TEntry",
            fieldbackground=entry_field_bg,
            background=entry_field_bg,
            foreground=TEXT_PRIMARY,
            bordercolor=BORDER_COLOR,
            lightcolor=BORDER_COLOR,
            darkcolor=BORDER_COLOR,
            padding=6,
        )
        self._style.map("Console.TEntry", fieldbackground=[("focus", entry_field_bg)], foreground=[("disabled", TEXT_MUTED)])

        self._style.configure("TNotebook", background=WINDOW_BG, borderwidth=0, padding=0)
        self._style.configure("TNotebook.Tab", background=WINDOW_BG, foreground=TEXT_MUTED, padding=(12, 8))
        self._style.map(
            "TNotebook.Tab",
            background=[("selected", PANEL_BG), ("!disabled", WINDOW_BG)],
            foreground=[("selected", TEXT_PRIMARY), ("!disabled", TEXT_MUTED)],
        )

    def _schedule_update(self) -> None:
        self.root.after(20, self._update)

    def _update(self) -> None:
        snapshot = self.joystick.snapshot()
        telemetry = self.telemetry.snapshot()
        self._update_controller_panel(snapshot)
        self._update_telemetry_panel(telemetry)
        self._maybe_send_packet(snapshot)
        self._schedule_update()

    def _update_controller_panel(self, snapshot: ControllerSnapshot) -> None:
        if snapshot.connected:
            label = f"Controller: {snapshot.name}"
            summary = (
                f"Axes available: {len(snapshot.axes)}\n"
                f"Buttons available: {len(snapshot.buttons)}\n"
                "Visual indicators are currently disabled."
            )
        else:
            label = "Controller: searching..."
            summary = "Waiting for controller connection."
        self.device_label_var.set(label)
        self.controller_summary_var.set(summary)

    def _update_telemetry_panel(self, snapshot: TelemetrySnapshot) -> None:
        self.voltage_var.set(self._format_float(snapshot.voltage))
        self.current_var.set(self._format_float(snapshot.current))
        self.temperature_var.set(self._format_float(snapshot.temperature))
        if snapshot.last_update is None:
            self.last_update_var.set("No telemetry received")
        else:
            elapsed = time.time() - snapshot.last_update
            self.last_update_var.set(f"Updated {elapsed:.1f}s ago")

    def _send_console_command(self, command: str) -> Optional[str]:
        data = command.encode("utf-8")
        try:
            self._command_socket.sendto(data, COMMAND_DESTINATION)
        except OSError as exc:
            return f"send failed: {exc}"
        return None

    def _maybe_send_packet(self, snapshot: ControllerSnapshot) -> None:
        now = time.monotonic()
        interval = 1.0 / (SEND_RATE_HZ if snapshot.connected else IDLE_RATE_HZ)
        if now - self._last_tx < interval:
            return

        axes = (snapshot.axes + [0.0] * MAX_DISPLAY_AXES)[:MAX_DISPLAY_AXES]
        buttons = snapshot.buttons
        armed = False
        if HOLD_BUTTON_INDEX < len(buttons):
            armed = bool(buttons[HOLD_BUTTON_INDEX])

        axes_i8 = [quantize_axis(value, DEADZONE) for value in axes]
        mask = 0
        for i in range(min(16, len(buttons))):
            if buttons[i]:
                mask |= 1 << i

        packet = build_full_packet(self._seq, armed, axes_i8, mask)
        try:
            self._udp_socket.sendto(packet, UDP_DESTINATION)
        except OSError:
            pass
        self._seq = (self._seq + 1) & 0xFF
        self._last_tx = now

    @staticmethod
    def _format_float(value: Optional[float]) -> str:
        if value is None:
            return "--"
        return f"{value:.2f}"

    def _on_close(self) -> None:
        self.joystick.stop()
        self.telemetry.stop()
        try:
            self._command_socket.close()
        except OSError:
            pass
        self.root.after(100, self.root.destroy)


# --- Entrypoint ------------------------------------------------------------------

def main() -> None:
    joystick = JoystickWorker()
    telemetry = TelemetryListener()
    joystick.start()
    telemetry.start()

    root = tk.Tk()
    app = ControlStationGUI(root, joystick, telemetry)

    try:
        root.mainloop()
    finally:
        joystick.stop()
        telemetry.stop()
        joystick.join(1.0)
        telemetry.join(1.0)


if __name__ == "__main__":
    main()
