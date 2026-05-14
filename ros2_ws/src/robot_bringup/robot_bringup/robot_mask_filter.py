#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from sensor_msgs_py import point_cloud2


class RobotMaskFilter(Node):
    def __init__(self):
        super().__init__('robot_mask_filter')

        # Topics
        self.declare_parameter('input_topic', '/cloud_registered_body')
        self.declare_parameter('output_topic', '/cloud_filtered')
        self.declare_parameter('marker_topic', '/robot_mask_marker')

        # Frame to use for marker. Usually leave this equal to incoming cloud frame.
        self.declare_parameter('marker_frame', 'body')

        # Robot mask dimensions in meters
        # Example: robot extends 36 in behind the lidar
        self.declare_parameter('robot_back_m', 2.0)   # 36 in
        self.declare_parameter('y_min_m', -0.2540)       # -10 in
        self.declare_parameter('y_max_m', 0.6080)        # +20 in
        self.declare_parameter('z_min_m', -1.0)          # optional vertical clipping
        self.declare_parameter('z_max_m',  0.0)
        self.declare_parameter('margin_m', 0.05)         # 5 cm safety margin

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        self.marker_frame = self.get_parameter('marker_frame').value

        self.robot_back_m = float(self.get_parameter('robot_back_m').value)
        self.y_min_m = float(self.get_parameter('y_min_m').value)
        self.y_max_m = float(self.get_parameter('y_max_m').value)
        self.z_min_m = float(self.get_parameter('z_min_m').value)
        self.z_max_m = float(self.get_parameter('z_max_m').value)
        self.margin_m = float(self.get_parameter('margin_m').value)

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            10
        )

        self.cloud_pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)

        self.get_logger().info(f'Listening on {input_topic}, publishing {output_topic}')

    def cloud_callback(self, msg: PointCloud2):
        # Read x, y, z (and ignore everything else for now)
        # ROS 2 Python utilities support reading PointCloud2 directly. :contentReference[oaicite:2]{index=2}
        pts = np.array([
            [p[0], p[1], p[2]]
            for p in point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )
        ], dtype=np.float32)

        if pts.size == 0:
            self.cloud_pub.publish(msg)
            self.publish_marker(msg.header.frame_id)
            return

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]

        # Robot box bounds
        x_min = -self.robot_back_m - self.margin_m
        x_max = 0.0 + self.margin_m
        y_min = self.y_min_m - self.margin_m
        y_max = self.y_max_m + self.margin_m
        z_min = self.z_min_m
        z_max = self.z_max_m

        # Mask points inside the robot body box
        mask_robot = (
            (x > x_min) & (x < x_max) &
            (y > y_min) & (y < y_max) &
            (z > z_min) & (z < z_max)
        )

        pts_filtered = pts[~mask_robot]

        filtered_msg = self.numpy_to_cloud(
            pts_filtered,
            header=msg.header
        )

        self.cloud_pub.publish(filtered_msg)
        self.publish_marker(msg.header.frame_id)

    def numpy_to_cloud(self, pts: np.ndarray, header: Header) -> PointCloud2:
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        if pts.shape[0] == 0:
            return point_cloud2.create_cloud(header, fields, [])

        return point_cloud2.create_cloud(header, fields, pts.tolist())

    def publish_marker(self, frame_id: str):
        # Use cloud frame if available; otherwise fallback to parameter
        marker_frame = frame_id if frame_id else self.marker_frame

        x_min = -self.robot_back_m - self.margin_m
        x_max = 0.0 + self.margin_m
        y_min = self.y_min_m - self.margin_m
        y_max = self.y_max_m + self.margin_m
        z_min = self.z_min_m
        z_max = self.z_max_m

        marker = Marker()
        marker.header.frame_id = marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_mask'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = (x_min + x_max) / 2.0
        marker.pose.position.y = (y_min + y_max) / 2.0
        marker.pose.position.z = (z_min + z_max) / 2.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = max(x_max - x_min, 0.001)
        marker.scale.y = max(y_max - y_min, 0.001)
        marker.scale.z = max(z_max - z_min, 0.001)

        marker.color.a = 0.30
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMaskFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
