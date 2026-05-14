#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class LioBaseFootprintTF(Node):
    def __init__(self):
        super().__init__('lio_base_footprint_tf')

        # Parameters
        self.declare_parameter('odom_topic', '/aft_mapped_to_init')  # or /lio_odom, etc.
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_footprint_frame', 'base_footprint')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_footprint_frame = self.get_parameter('base_footprint_frame').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            50
        )
        self.get_logger().info(
            f'Subscribing to LIO odom on "{odom_topic}", publishing TF {self.odom_frame} -> {self.base_footprint_frame}'
        )

    def odom_callback(self, msg: Odometry):
        # Extract planar pose: x, y, yaw from full 3D odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        # Convert quaternion to yaw (Z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_footprint_frame

        # Planar translation: z = 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Planar rotation: roll = pitch = 0, yaw from above
        half_yaw = 0.5 * yaw
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LioBaseFootprintTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
