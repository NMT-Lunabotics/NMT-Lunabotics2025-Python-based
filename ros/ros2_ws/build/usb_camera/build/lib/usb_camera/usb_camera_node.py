import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    rclpy.init()
    node = Node('usb_camera_node')
    pub = node.create_publisher(Image, 'image_raw', 10)
    bridge = CvBridge()

    # Open Pi camera (or V4L2 camera)
    cap = cv2.VideoCapture('/dev/video20')  # change device as needed

    if not cap.isOpened():
        node.get_logger().error('Cannot open camera /dev/video20')
        return

    node.get_logger().info('Publishing camera feed on /image_raw')

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                continue
            msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()
