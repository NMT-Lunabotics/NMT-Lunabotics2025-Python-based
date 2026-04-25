#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import glob

class VideoDeviceCounter(Node):
    def __init__(self):
        super().__init__('count_streams')

        self.pub = self.create_publisher(Int32, '/count_streams', 10)
        self.timer = self.create_timer(10.0, self.update)

    def update(self):
        devices = glob.glob('/dev/video*')
        devices = [d for d in devices if d[-1].isdigit()]
        msg = Int32()
        msg.data = len(devices)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = VideoDeviceCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()