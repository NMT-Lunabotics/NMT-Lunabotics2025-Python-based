#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class LocalCostmapSupport(Node):
    def __init__(self):
        super().__init__('local_costmap_support')

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static laser transform ONCE
        self.publish_laser_tf()

        # Timer for odometry + dynamic TF
        self.timer = self.create_timer(0.05, self.publish_odom)  # 20 Hz

        self.get_logger().info('Local costmap support node started')

    def publish_laser_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'laser'

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(tf)

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        # ---- Odometry message ----
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.orientation.w = 1.0

        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        # ---- Dynamic TF: odom → base_link ----
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = LocalCostmapSupport()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
