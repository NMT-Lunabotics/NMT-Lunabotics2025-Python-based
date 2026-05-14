#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class PointCloudFilter(Node):

    def __init__(self):
        super().__init__('pointcloud_filter')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            10)

        self.publisher = self.create_publisher(
            PointCloud2,
            '/filtered_obstacles',
            10)

        self.get_logger().info("Point cloud floor/ceiling filter started")


    def cloud_callback(self, msg):

        points = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)

        filtered_points = []

        for p in points:

            x = p[0]
            y = p[1]
            z = p[2]

            # remove ceiling
            if z > 1.5:
                continue

            # remove floor
            if z < -0.2:
                continue

            # keep points within navigation range
            if 0.2 < x < 3.0:
                filtered_points.append([x, y, z])

        obstacle_count = len(filtered_points)

        if obstacle_count > 200:
            self.get_logger().info(f"Obstacle points: {obstacle_count}")

        filtered_cloud = pc2.create_cloud_xyz32(msg.header, filtered_points)

        self.publisher.publish(filtered_cloud)


def main(args=None):

    rclpy.init(args=args)

    node = PointCloudFilter()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()