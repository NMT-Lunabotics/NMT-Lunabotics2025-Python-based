#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomAndFrames(Node):
    def __init__(self):
        super().__init__('odom_and_frames')

        # ---------- Publishers ----------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # ---------- Robot state ----------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Simple velocity model (replace later if desired)
        self.v = 0.1   # m/s
        self.w = 0.0   # rad/s

        self.last_time = self.get_clock().now()

        # ---------- Timers ----------
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        # ---------- Publish static frames ONCE ----------
        self.publish_static_frames()

    # -------------------------------------------------
    def publish_static_frames(self):
        now = self.get_clock().now().to_msg()
        tfs = []

        def add(parent, child):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            tfs.append(t)

        add('base_link', 'laser')
        add('base_link', 'camera_link')
        add('camera_link', 'camera_color_optical_frame')
        add('camera_link', 'camera_depth_optical_frame')
        add('camera_link', 'imu_link')

        self.static_broadcaster.sendTransform(tfs)

    # -------------------------------------------------
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Integrate pose
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # ---------- Odometry ----------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

        # ---------- TF odom → base_link ----------
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomAndFrames()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
