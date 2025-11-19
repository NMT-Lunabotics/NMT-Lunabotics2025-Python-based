#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanPrinter(Node):
    def __init__(self):
        super().__init__('scan_printer')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        self.get_logger().info(f"Got scan with {len(msg.ranges)} points")

def main(args=None):
    rclpy.init(args=args)
    node = ScanPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
