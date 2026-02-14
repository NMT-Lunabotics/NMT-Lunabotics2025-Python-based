#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MultiCameraPublisher(Node):
    def __init__(self, fps=30):
        super().__init__('simple_camera_stream_node')
        # Raw camrea streams to publish
        self.cameras = [
            {'device': '/dev/video0', 'topic': '/camera/depth/image_raw'},
            {'device': '/dev/video4', 'topic': '/camera/color/image_raw'}
        ]
        # Usage variables
        self.bridge = CvBridge()
        self.frame_interval = 1.0 / fps
        self.publishers = []
        # For each of our camera streams publish it's respective topic
        for cam in self.cameras:
            cap = cv2.VideoCapture(cam['device'])
            if cap.isOpened():
                self.publishers.append({'cap': cap,'topic': cam['topic'],'pub': self.create_publisher(Image, cam['topic'], 10)})
        self.timer = self.create_timer(self.frame_interval, self.publish_frames)

    def publish_frames(self):
        for cam in self.publishers:
            ret, frame = cam['cap'].read()
            if not ret: continue
            cam['pub'].publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher(fps=10)
    rclpy.spin(node)
    for cam in node.publishers:
        cam['cap'].release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
