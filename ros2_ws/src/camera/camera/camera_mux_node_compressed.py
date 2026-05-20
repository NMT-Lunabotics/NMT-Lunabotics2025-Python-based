#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from robot_interfaces.msg import Camera
import cv2
import numpy as np

class CameraMux(Node):
    def __init__(self):
        super().__init__('camera_mux_node')
        self.cameras = {
            0: 'camera0/image_raw/compressed',
            1: 'camera1/image_raw/compressed',
            2: 'camera2/image_raw/compressed',
        }
        self.direct_mode = True
        self.camera_indexes = [0,1]
        self.index_cycle_state=0

        self.active_index = 0
        self.last_index = None

        publisher_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1,durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(CompressedImage, '/camera/stream', publisher_qos)
        self.latest_frames = {}
        if self.direct_mode: self.camera_indexes = list(self.cameras.keys())

        self.declare_parameter('nav_stream', "-1")
        self.nav_stream = int(self.get_parameter('nav_stream').value)
        if self.nav_stream !=-1: self.cameras[self.nav_stream] = '/camera/camera/color/image_raw/compressed2'

        self.declare_parameter('output_fps', 5.0)
        self.output_fps = self.get_parameter('output_fps').value

        self.last_publish_time = self.get_clock().now()

        self.subs = []
        for idx, topic in self.cameras.items():
            sub_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
            def make_callback(index): return lambda msg: self.image_active(msg, index)
            self.subs.append(self.create_subscription(CompressedImage, topic, make_callback(idx), sub_qos))

        self.create_subscription(Camera, '/camera/toggle_view', self.toggle_stream, 10)
        self.timer = self.create_timer(1.0 / self.output_fps, self.publish_active)

        self.jpeg_quality = 30

    def toggle_stream(self, msg: Camera):
            if self.direct_mode:
                if msg.camera_view in self.cameras:
                    self.active_index = msg.camera_view
                    self.last_index = self.active_index
                return

    def image_active(self, msg, index):
        self.latest_frames[index] = msg

    def publish_active(self):
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_publish_time).nanoseconds / 1e9

        if time_since_last < (1.0 / self.output_fps): return
        if self.active_index not in self.latest_frames: return
        cam_id = self.camera_indexes[self.active_index]
        if cam_id not in self.latest_frames: return

        msg = self.latest_frames[cam_id]
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        ok, jpeg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            out = CompressedImage()
            out.header = msg.header
            out.format = "jpeg"
            out.data = jpeg.tobytes()
            self.pub.publish(out)

        self.last_publish_time = current_time

def main():
    rclpy.init()
    node = CameraMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
