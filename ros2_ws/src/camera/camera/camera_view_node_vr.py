#!/usr/bin/env python3
from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


DISCOVERY_MAGIC = b"F310_DISCOVERY_V1"
DISCOVERY_PORT = 11010
TELEMETRY_PORT = 10000
TELEMETRY_RATE_HZ = 2.0
CAMERA_HTTP_PORT = 8081
CAMERA_JPEG_QUALITY = 80


class DriverStationTracker:
    def __init__(self, initial: Optional[str] = None) -> None:
        self._lock = threading.Lock()
        self._addr = initial

    def update(self, ip: str) -> None:
        if not ip:
            return
        with self._lock:
            self._addr = ip

    def current(self) -> Optional[str]:
        with self._lock:
            return self._addr


class DiscoveryResponder:
    def __init__(
        self,
        listen_port: int,
        telemetry_port: int,
        tracker: DriverStationTracker,
    ) -> None:
        self._listen_port = listen_port
        self._tracker = tracker
        self._reply = json.dumps(
            {
                "role": "camera_view",
                "udp_port": 0,
                "command_port": 0,
                "telemetry_port": telemetry_port,
            }
        ).encode("utf-8")
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="DiscoveryResponder", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self._listen_port))
            sock.settimeout(0.5)
        except OSError:
            return

        with sock:
            while not self._stop_event.is_set():
                try:
                    data, addr = sock.recvfrom(128)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if data.strip() != DISCOVERY_MAGIC:
                    continue
                self._tracker.update(addr[0])
                try:
                    sock.sendto(self._reply, addr)
                except OSError:
                    continue


class TelemetryBroadcaster:
    def __init__(
        self,
        tracker: DriverStationTracker,
        telemetry_port: int,
        payload_factory: Callable[[Optional[str]], Optional[bytes]],
        interval: float,
    ) -> None:
        self._tracker = tracker
        self._telemetry_port = telemetry_port
        self._payload_factory = payload_factory
        self._interval = max(0.2, interval)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="TelemetryBroadcaster", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        with sock:
            while not self._stop_event.is_set():
                target = self._tracker.current()
                payload = self._payload_factory(target)
                if target and payload:
                    try:
                        sock.sendto(payload, (target, self._telemetry_port))
                    except OSError:
                        pass
                self._stop_event.wait(self._interval)


def guess_advertised_host(driver_station_ip: Optional[str], override: Optional[str] = None) -> Optional[str]:
    if override:
        return override
    if not driver_station_ip:
        return None

    probe: Optional[socket.socket] = None
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect((driver_station_ip, 9))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        if probe is not None:
            probe.close()


def auto_local_ip() -> str:
    probe: Optional[socket.socket] = None
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"
    finally:
        if probe is not None:
            probe.close()


class CameraRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/video":
            self._serve_mjpeg()
            return
        if self.path not in ("/", "/frame"):
            self.send_error(404, "Not Found")
            return

        frame = self.server.video_source.latest_jpeg()  # type: ignore[attr-defined]
        if frame is None:
            self.send_error(503, "Camera frame unavailable")
            return

        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.end_headers()
        self.wfile.write(frame)

    def _serve_mjpeg(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        while True:
            frame = self.server.video_source.latest_jpeg()  # type: ignore[attr-defined]
            if frame is None:
                time.sleep(0.03)
                continue
            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            time.sleep(0.03)

    def log_message(self, fmt: str, *args: object) -> None:
        pass


class CameraHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], video_source: "CameraViewNode") -> None:
        self.video_source = video_source
        super().__init__(address, CameraRequestHandler)


class CameraViewNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_view_node")

        self.declare_parameter("image_topic", "/camera/stream")
        self.declare_parameter("window_width", 800)
        self.declare_parameter("window_height", 600)
        self.declare_parameter("fullscreen", False)
        self.declare_parameter("use_gui", True)
        self.declare_parameter("webstream", True)
        self.declare_parameter("web_port", CAMERA_HTTP_PORT)

        self.use_gui = bool(self.get_parameter("use_gui").value)
        self.webstream = bool(self.get_parameter("webstream").value)
        self.latest_frame = None
        self.latest_frame_width = 0
        self.latest_frame_height = 0
        self.latest_frame_time = 0.0
        self.latest_frame_count = 0
        self.start_time = time.monotonic()
        self.lock = threading.Lock()

        self.last_display_time = 0.0
        self.fps_limit = 20.0

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = "Camera View"
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.web_port = int(self.get_parameter("web_port").value)
        self.bind_host = auto_local_ip()
        self.advertise_host: Optional[str] = self.bind_host

        self.tracker = DriverStationTracker()
        self.discovery = DiscoveryResponder(DISCOVERY_PORT, TELEMETRY_PORT, self.tracker)
        self.telemetry = TelemetryBroadcaster(
            self.tracker,
            TELEMETRY_PORT,
            self._build_payload,
            1.0 / TELEMETRY_RATE_HZ,
        )

        self.create_subscription(CompressedImage, self.image_topic, self.image_callback, 10)

        if self.use_gui:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            if self.fullscreen:
                cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            else:
                cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
            self.create_timer(0.03, self.update_display)

        if self.webstream:
            self.http_server = CameraHTTPServer((self.bind_host, self.web_port), self)
            self.http_thread = threading.Thread(
                target=self.http_server.serve_forever,
                name="CameraHTTP",
                daemon=True,
            )
            self.http_thread.start()
            self.discovery.start()
            self.telemetry.start()
            self.get_logger().info(
                f"Serving camera JPEG frames at http://{self.bind_host}:{self.web_port}/frame"
            )

    def image_callback(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), CAMERA_JPEG_QUALITY])
        if not ok:
            return

        height, width = frame.shape[:2]
        with self.lock:
            self.latest_frame = frame
            self.latest_jpeg_frame = jpeg.tobytes()
            self.latest_frame_width = width
            self.latest_frame_height = height
            self.latest_frame_time = time.monotonic()
            self.latest_frame_count += 1

    def latest_jpeg(self) -> Optional[bytes]:
        with self.lock:
            frame = getattr(self, "latest_jpeg_frame", None)
            return None if frame is None else bytes(frame)

    def update_display(self) -> None:
        now = time.time()
        if now - self.last_display_time < 1.0 / self.fps_limit:
            return
        self.last_display_time = now

        with self.lock:
            if self.latest_frame is None:
                return
            display = self.latest_frame.copy()
        if not self.fullscreen:
            display = cv2.resize(display, (self.window_width, self.window_height), interpolation=cv2.INTER_LINEAR)

        cv2.imshow(self.window_name, display)
        cv2.waitKey(1)

    def _build_payload(self, driver_station_ip: Optional[str]) -> Optional[bytes]:
        camera_host = self.advertise_host or guess_advertised_host(driver_station_ip)
        camera_url = None
        if camera_host:
            camera_url = f"http://{camera_host}:{self.web_port}/frame"

        elapsed = max(1e-6, time.monotonic() - self.start_time)
        with self.lock:
            frame_age = None if self.latest_frame_time == 0.0 else round(time.monotonic() - self.latest_frame_time, 3)
            payload = {
                "camera_url": camera_url,
                "video_width": self.latest_frame_width,
                "video_height": self.latest_frame_height,
                "video_quality": CAMERA_JPEG_QUALITY,
                "video_target_fps": self.fps_limit,
                "video_low_latency": True,
                "video_actual_fps": round(self.latest_frame_count / elapsed, 2),
                "video_frame_age": frame_age,
            }
        return json.dumps(payload).encode("utf-8")

    def destroy_node(self) -> bool:
        if self.webstream:
            self.telemetry.stop()
            self.discovery.stop()
            self.http_server.shutdown()
            self.http_server.server_close()
            self.http_thread.join(timeout=2.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CameraViewNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
