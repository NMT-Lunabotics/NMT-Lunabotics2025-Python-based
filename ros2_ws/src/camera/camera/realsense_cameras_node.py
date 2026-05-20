#!/usr/bin/env python3

import rclpy, threading, yaml, cv2, os, time
from rclpy.node import Node
import pyrealsense2 as realsense
import numpy as numpy_array
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, Imu
from cv_bridge import CvBridge


def realsense_to_ros_frame(vector_input):
    axis_x, axis_y, axis_z = vector_input
    return numpy_array.array([-axis_x, -axis_y, -axis_z], dtype=numpy_array.float32)


class RealsenseCamera:
    def __init__(self, ros_node, camera_configuration):
        self.ros_node = ros_node
        self.camera_configuration = camera_configuration

        self.camera_id = camera_configuration["id"]
        self.device_uuid = camera_configuration["uuid"]

        self.pipeline = realsense.pipeline()
        self.pipeline_config = realsense.config()
        self.pipeline_config.enable_device(self.device_uuid)

        stream_config = camera_configuration["stream"]
        stream_width = stream_config["width"]
        stream_height = stream_config["height"]
        stream_fps = stream_config["fps"]
        self.stream_enabled = stream_config.get("enabled", True)

        self.target_fps = stream_fps
        self.min_interval = 1.0 / float(self.target_fps)
        self.last_publish_time = 0.0

        self.pipeline_config.enable_stream(realsense.stream.color, stream_width, stream_height, realsense.format.bgr8, stream_fps)
        if self.stream_enabled: self.pipeline_config.enable_stream(realsense.stream.depth, stream_width, stream_height, realsense.format.z16, stream_fps)

        imu_config = camera_configuration.get("imu", {})
        self.imu_enabled = imu_config.get("enabled", False)

        if self.imu_enabled:
            self.pipeline_config.enable_stream(realsense.stream.gyro)
            self.pipeline_config.enable_stream(realsense.stream.accel)

        try:
            self.profile = self.pipeline.start(self.pipeline_config)
        except RuntimeError as e:
            self.ros_node.get_logger().error(f"Camera {self.device_uuid} failed to start: {e}")
            self.running_flag = False
            return

        device = self.profile.get_device()
        for sensor in device.query_sensors():
            sensor.set_option(realsense.option.frames_queue_size, 1)

        self.bridge = CvBridge()

        base_topic = f"/camera{self.camera_id}"

        if self.stream_enabled:
            self.rgb_pub = ros_node.create_publisher(Image, base_topic + "/color/image_raw", 10)
            self.rgb_info_pub = ros_node.create_publisher(CameraInfo, base_topic + "/color/camera_info", 10)

            self.depth_pub = ros_node.create_publisher(Image, base_topic + "/depth/image_raw", 10)
            self.depth_info_pub = ros_node.create_publisher(CameraInfo, base_topic + "/depth/camera_info", 10)
        
        self.rgb_comp_pub = ros_node.create_publisher(CompressedImage, base_topic + "/color/image_raw/compressed", 10)

        if self.imu_enabled: self.imu_pub = ros_node.create_publisher(Imu, base_topic + "/imu/data", 10)

        self.color_intrinsics = self.profile.get_stream(realsense.stream.color).as_video_stream_profile().get_intrinsics()
        self.depth_intrinsics = self.profile.get_stream(realsense.stream.depth).as_video_stream_profile().get_intrinsics()

        self.running_flag = True
        self.thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.thread.start()

    def build_camera_info(self, intrinsics_data, timestamp_message):
        crop = self.camera_configuration["stream"]["crop"]

        camera_info = CameraInfo()
        camera_info.width = crop["width"]
        camera_info.height = crop["height"]
        camera_info.distortion_model = "plumb_bob"

        focal_x = intrinsics_data.fx
        focal_y = intrinsics_data.fy
        center_x = intrinsics_data.ppx - crop["x"]
        center_y = intrinsics_data.ppy - crop["y"]

        camera_info.k = [focal_x, 0.0, center_x, 0.0, focal_y, center_y, 0.0, 0.0, 1.0]
        camera_info.d = list(intrinsics_data.coeffs)
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [focal_x, 0.0, center_x, 0.0, 0.0, focal_y, center_y, 0.0, 0.0, 0.0, 1.0, 0.0]

        camera_info.header.stamp = timestamp_message
        camera_info.header.frame_id = f"camera{self.camera_id}"
        return camera_info

    def processing_loop(self):

        crop = self.camera_configuration["stream"]["crop"]
        compression = self.camera_configuration["rgb_compressed"]

        while rclpy.ok() and self.running_flag:
            try:
                frames = self.pipeline.wait_for_frames()
            except:
                continue

            timestamp = self.ros_node.get_clock().now().to_msg()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            gyro_data = None
            accel_data = None

            if self.imu_enabled:
                for frame in frames:
                    if not frame.is_motion_frame(): continue
                    motion = frame.as_motion_frame().get_motion_data()
                    if frame.get_profile().stream_type() == realsense.stream.gyro:
                        gyro_data = numpy_array.array([motion.x, motion.y, motion.z], dtype=numpy_array.float32)
                    elif frame.get_profile().stream_type() == realsense.stream.accel:
                        accel_data = numpy_array.array([motion.x, motion.y, motion.z], dtype=numpy_array.float32)

                if gyro_data is not None or accel_data is not None:
                    imu_msg = Imu()
                    imu_msg.header.stamp = timestamp
                    imu_msg.header.frame_id = f"camera{self.camera_id}"

                    if gyro_data is not None:
                        g = realsense_to_ros_frame(gyro_data)
                        imu_msg.angular_velocity.x = float(g[0])
                        imu_msg.angular_velocity.y = float(g[1])
                        imu_msg.angular_velocity.z = float(g[2])

                    if accel_data is not None:
                        a = realsense_to_ros_frame(accel_data)
                        imu_msg.linear_acceleration.x = float(a[0])
                        imu_msg.linear_acceleration.y = float(a[1])
                        imu_msg.linear_acceleration.z = float(a[2])

                    imu_msg.orientation_covariance[0] = -1
                    self.imu_pub.publish(imu_msg)

            x, y = crop["x"], crop["y"]
            w, h = crop["width"], crop["height"]

            if color_frame:
                color_image = numpy_array.asanyarray(color_frame.get_data())
                color_image = color_image[y:y + h, x:x + w]
                color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")

                color_msg.header.stamp = timestamp
                color_msg.header.frame_id = f"camera{self.camera_id}"

                now = time.time()
                if now - self.last_publish_time < self.min_interval:
                    continue
                self.last_publish_time = now

                if self.stream_enabled:
                    self.rgb_pub.publish(color_msg)
                    self.rgb_info_pub.publish(self.build_camera_info(self.color_intrinsics, timestamp))

                resized = cv2.resize(color_image, (compression["width"], compression["height"]))
                ok, jpeg = cv2.imencode(".jpg", resized, [int(cv2.IMWRITE_JPEG_QUALITY), compression["jpeg_quality"]])

                if ok:
                    comp_msg = CompressedImage()
                    comp_msg.header = color_msg.header
                    comp_msg.format = "jpeg"
                    comp_msg.data = jpeg.tobytes()
                    self.rgb_comp_pub.publish(comp_msg)

            if self.stream_enabled and depth_frame is not None:
                depth_image = numpy_array.asanyarray(depth_frame.get_data())
                depth_image = depth_image[y:y + h, x:x + w]

                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "mono16")
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = f"camera{self.camera_id}"

                self.depth_pub.publish(depth_msg)
                self.depth_info_pub.publish(self.build_camera_info(self.depth_intrinsics, timestamp))

    def stop(self):
        self.running_flag = False
        self.pipeline.stop()


class RealsenseMultiNode(Node):
    def __init__(self):
        super().__init__("realsense_node")

        config_path = os.path.join(get_package_share_directory("camera"),"config","realsense_cameras.yaml")
        with open(config_path, "r") as file:
            config_data = yaml.safe_load(file)

        self.cameras = []
        for cam in config_data["cameras"]:
            try:
                self.cameras.append(RealsenseCamera(self, cam))
            except Exception as e:
                self.get_logger().error(str(e))

    def destroy_node(self):
        for cam in self.cameras:
            cam.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = RealsenseMultiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()