#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_interfaces.msg import Camera

class CameraMux(Node):
    def __init__(self):
        super().__init__('camera_mux_node')
        # Cameras to switch between
        self.cameras = {
            0: 'camera0/image_raw',
            1: 'camera1/image_raw',
        }

        self.active_index = 0
        self.last_index = None
        self.pub = self.create_publisher(Image,'/camera/stream',10)

        self.subs = []
        for idx, topic in self.cameras.items():
            self.subs.append(self.create_subscription(Image,topic,lambda msg, i=idx: self.image_active(msg, i),10))

        # Ensure toggle topic exists and has a default value
        self.create_subscription(Camera, '/camera/toggle_view', self.toggle_stream, 5)
        #toggle_pub = self.create_publisher(Camera, '/camera/toggle_view', 5)
        #default_msg = Camera()
        #default_msg.camera_view = 0
        #toggle_pub.publish(default_msg)

    # Switch camera streams
    def toggle_stream(self, msg: Camera):
        if msg.camera_view in self.cameras:
            self.active_index = msg.camera_view
            if self.active_index != self.active_index: self.get_logger().info(f'Switched to camera {msg.camera_view}')
            self.last_index = self.active_index

    # Publish active camera stream to final stream topic
    def image_active(self, msg, index):
        if index == self.active_index: self.pub.publish(msg)

def main():
    rclpy.init()
    node = CameraMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
