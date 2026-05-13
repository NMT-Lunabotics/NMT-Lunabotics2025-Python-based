#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
import os
import math
import shutil

from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint')

        package_share_directory = get_package_share_directory('point_navigation')
        default_file_path = os.path.join(package_share_directory, 'config', 'waypoint.yaml')

        runtime_directory = os.path.expanduser('~/Documents/NMT-Lunabotics2025-Python-based/ros2_ws/config_runtime')
        os.makedirs(runtime_directory, exist_ok=True)

        self.file_path = os.path.join(runtime_directory, 'waypoints.yaml')

        if not os.path.exists(self.file_path):
            shutil.copy(default_file_path, self.file_path)

        self.nav = BasicNavigator()

        self.waypoints = {}
        self.load()

        self.odom_received = False
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_yaw = 0.0

        self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)

        self.declare_parameter("target_name", "berm")
        self.declare_parameter("nav_target_name", "berm")

        self.create_service(SetBool, 'save_target_location', self.save_target_location)
        self.create_service(SetBool, 'navigation_goal_target', self.navigation_goal_target_cb)

    def odom_callback(self, odometry_message):
        self.odom_received = True

        self.latest_x = odometry_message.pose.pose.position.x
        self.latest_y = odometry_message.pose.pose.position.y

        quaternion = odometry_message.pose.pose.orientation

        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        self.latest_yaw = math.atan2(siny_cosp, cosy_cosp)

    def load(self):
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as file:
                self.waypoints = yaml.safe_load(file) or {}
        else:
            self.waypoints = {}

    def save(self):
        with open(self.file_path, 'w') as file:
            yaml.dump(self.waypoints, file)

    def save_target_location(self, request, response):
        waypoint_name = self.get_parameter("target_name").value

        if not self.odom_received:
            response.success = False
            response.message = "No odom received yet"
            return response

        self.waypoints[waypoint_name] = {
            'x': self.latest_x,
            'y': self.latest_y,
            'yaw': self.latest_yaw
        }

        self.save()

        response.success = True
        response.message = f"Saved {waypoint_name}"
        return response

    def navigation_goal_target_cb(self, request, response):
        waypoint_name = self.get_parameter("nav_target_name").value

        if waypoint_name not in self.waypoints:
            response.success = False
            response.message = "Not found"
            return response

        waypoint = self.waypoints[waypoint_name]

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(waypoint["x"])
        pose.pose.position.y = float(waypoint["y"])

        quaternion = self.quat_from_yaw(float(waypoint["yaw"]))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.nav.goToPose(pose)

        response.success = True
        response.message = f"Navigating to {waypoint_name}"
        return response

    def quat_from_yaw(self, yaw_angle):
        return (0.0, 0.0, math.sin(yaw_angle / 2.0), math.cos(yaw_angle / 2.0))


def main():
    rclpy.init()
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()