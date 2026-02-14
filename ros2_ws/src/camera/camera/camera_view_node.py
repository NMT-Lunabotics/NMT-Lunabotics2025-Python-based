#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_view_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/camera/rgb/image_raw/compressed',self.listener_callback,10)

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Feed", 800, 600)
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
