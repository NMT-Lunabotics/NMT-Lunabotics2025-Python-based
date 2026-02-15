#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from controller_input.msg import Camera
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_view_node')

        # Import needed class
        self.bridge = CvBridge()

        # Data to hold active camera topics and changes which can be switched to
        self.active_camera_topic = "/camera/rgb/img_raw0"
        self.cameras=["/camera/rgb/img_raw0","/camera/rgb/img_raw2"]

        # Create subscriptions for each camera feed
        for camera in self.cameras:
            # Camera callback to remember each topic
            def callback(msg, topic=camera):
                self.image_callback(msg, topic)

            # Store camera subscription
            setattr(self, f'sub_{camera.replace("/", "_")}', self.create_subscription(Image, camera, callback, 10))

        # Subscribe to toggle topic
        self.create_subscription(Camera, '/camera/toggle_view', self.toggle_callback, 10)


    def toggle_callback(self, msg: Camera):
        self.active_camera_topic=self.cameras[msg.camera_view]

    def image_callback(self, msg, topic):
        # Only display the active camera
        if topic != self.active_camera_topic: return
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
