#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import threading
import yaml
import cv2
import os

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge


class RealsenseCamera:
    def __init__(self, node, camera_config):
        self.node = node
        self.camera_config = camera_config

        self.camera_id = camera_config["id"]
        self.device_uuid = camera_config["uuid"]

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.device_uuid)

        stream_config = camera_config["stream"]
        width = stream_config["width"]
        height = stream_config["height"]
        fps = stream_config["fps"]

        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        imu_config = camera_config.get("imu", {})
        imu_enabled = imu_config.get("enabled", False)
        self.imu_enabled = imu_enabled

        if imu_enabled:
            self.config.enable_stream(rs.stream.gyro)
            self.config.enable_stream(rs.stream.accel)

        self.profile = self.pipeline.start(self.config)

        self.bridge = CvBridge()

        base = f"/camera{self.camera_id}"

        self.rgb_pub = node.create_publisher(Image, base + "/color/image_raw", 10)
        self.depth_pub = node.create_publisher(Image, base + "/depth/image_raw", 10)
        self.rgb_comp_pub = node.create_publisher(CompressedImage, base + "/color/image_raw/compressed", 10)

        self.rgb_info_pub = node.create_publisher(CameraInfo, base + "/color/camera_info", 10)
        self.depth_info_pub = node.create_publisher(CameraInfo, base + "/depth/camera_info", 10)

        self.gyro_pub = node.create_publisher(Vector3Stamped, base + "/imu/gyro", 10)
        self.accel_pub = node.create_publisher(Vector3Stamped, base + "/imu/accel", 10)

        self.color_intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def intrinsics_to_msg(self, intrinsics, stamp):
        msg = CameraInfo()
        msg.width = intrinsics.width
        msg.height = intrinsics.height
        msg.distortion_model = "plumb_bob"
        msg.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        msg.d = list(intrinsics.coeffs)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        msg.header.stamp = stamp
        return msg

    def loop(self):
        crop = self.camera_config["stream"]["crop"]
        comp = self.camera_config["rgb_compressed"]
        imu_config = self.camera_config.get("imu", {})

        imu_frequency = imu_config.get("frequency", 0)

        while rclpy.ok() and self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            except Exception:
                continue

            stamp = self.node.get_clock().now().to_msg()

            color = frames.get_color_frame()
            if color:
                color_img = np.asanyarray(color.get_data())

                x = crop["x"]
                y = crop["y"]
                w = crop["width"]
                h = crop["height"]

                color_img = color_img[y:y + h, x:x + w]

                img_msg = self.bridge.cv2_to_imgmsg(color_img, "bgr8")
                img_msg.header.stamp = stamp
                self.rgb_pub.publish(img_msg)

                self.rgb_info_pub.publish(self.intrinsics_to_msg(self.color_intrinsics, stamp))

                resized = cv2.resize(color_img, (comp["width"], comp["height"]))

                ok, jpeg = cv2.imencode(
                    ".jpg",
                    resized,
                    [int(cv2.IMWRITE_JPEG_QUALITY), comp["jpeg_quality"]]
                )

                if ok:
                    comp_msg = CompressedImage()
                    comp_msg.header = img_msg.header
                    comp_msg.format = "jpeg"
                    comp_msg.data = jpeg.tobytes()
                    self.rgb_comp_pub.publish(comp_msg)

            depth = frames.get_depth_frame()
            if depth:
                depth_img = np.asanyarray(depth.get_data())

                depth_msg = self.bridge.cv2_to_imgmsg(depth_img, "mono16")
                depth_msg.header.stamp = stamp
                self.depth_pub.publish(depth_msg)

                self.depth_info_pub.publish(self.intrinsics_to_msg(self.depth_intrinsics, stamp))

            if self.imu_enabled:
                for frame in frames:
                    if not frame.is_motion_frame():
                        continue

                    motion = frame.as_motion_frame().get_motion_data()

                    imu_msg = Vector3Stamped()
                    imu_msg.header.stamp = stamp
                    imu_msg.vector.x = motion.x
                    imu_msg.vector.y = motion.y
                    imu_msg.vector.z = motion.z

                    stream_name = frame.get_profile().stream_name()

                    if stream_name == "Gyro":
                        self.gyro_pub.publish(imu_msg)

                    elif stream_name == "Accel":
                        self.accel_pub.publish(imu_msg)

    def stop(self):
        self.running = False
        self.pipeline.stop()


class RealsenseMultiNode(Node):
    def __init__(self):
        super().__init__("realsense_cameras_node")

        package_path = get_package_share_directory("camera")
        config_path = os.path.join(package_path, "config", "realsense_cameras.yaml")

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        self.cameras = [
            RealsenseCamera(self, cam)
            for cam in config["cameras"]
        ]

    def destroy_node(self):
        for c in self.cameras:
            c.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = RealsenseMultiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()