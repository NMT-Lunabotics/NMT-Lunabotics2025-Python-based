#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CameraViewNode(Node):
    def __init__(self):
        super().__init__('camera_view_node')
        # Display settings
        self.latest_image = None
        self.declare_parameter('image_topic', '/camera/stream')
        self.declare_parameter('window_name', 'Camera View')
        self.declare_parameter('window_width', 800)
        self.declare_parameter('window_height', 600)
        self.declare_parameter('fullscreen', False)
        self.image_topic = self.get_parameter('image_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value
        self.fullscreen = self.get_parameter('fullscreen').value

        # Setup screen handlers 
        self.create_subscription(CompressedImage,self.image_topic,self.image_callback,10)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if self.fullscreen: cv2.setWindowProperty(self.window_name,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
        else: cv2.resizeWindow(self.window_name,self.window_width,self.window_height)
        self.create_timer(0.03, self.update_display)

    # Update screen image
    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Update screen display, use cv2 to ensure user can fullscreen camera feed and resize it
    def update_display(self):
        if self.latest_image is None: return
        display = self.latest_image
        if not self.fullscreen: display = cv2.resize(display,(self.window_width, self.window_height),interpolation=cv2.INTER_LINEAR)
        cv2.imshow(self.window_name, display)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraViewNode()
    try: rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()