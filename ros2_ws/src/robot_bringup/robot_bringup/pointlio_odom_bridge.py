#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros import TransformException


class PointLioOdomBridge(Node):
    def __init__(self):
        super().__init__('pointlio_odom_bridge')

        self.declare_parameter('source_parent', 'camera_init')
        self.declare_parameter('source_child', 'aft_mapped')
        self.declare_parameter('target_parent', 'odom')
        self.declare_parameter('target_child', 'base_link')
        self.declare_parameter('publish_rate', 30.0)

        self.source_parent = self.get_parameter('source_parent').value
        self.source_child = self.get_parameter('source_child').value
        self.target_parent = self.get_parameter('target_parent').value
        self.target_child = self.get_parameter('target_child').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Bridging {self.source_parent}->{self.source_child} '
            f'to {self.target_parent}->{self.target_child}'
        )

    def timer_callback(self):
        try:
            src_tf = self.tf_buffer.lookup_transform(
                self.source_parent,
                self.source_child,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not lookup {self.source_parent}->{self.source_child}: {ex}',
                throttle_duration_sec=2.0
            )
            return

        out_tf = TransformStamped()
        out_tf.header.stamp = src_tf.header.stamp
        out_tf.header.frame_id = self.target_parent
        out_tf.child_frame_id = self.target_child

        out_tf.transform.translation.x = src_tf.transform.translation.x
        out_tf.transform.translation.y = src_tf.transform.translation.y
        out_tf.transform.translation.z = src_tf.transform.translation.z

        out_tf.transform.rotation.x = src_tf.transform.rotation.x
        out_tf.transform.rotation.y = src_tf.transform.rotation.y
        out_tf.transform.rotation.z = src_tf.transform.rotation.z
        out_tf.transform.rotation.w = src_tf.transform.rotation.w

        self.tf_broadcaster.sendTransform(out_tf)


def main():
    rclpy.init()
    node = PointLioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



