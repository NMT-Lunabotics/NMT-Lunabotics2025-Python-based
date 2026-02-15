#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyudev
import cv2

class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera_stream_node')
        # Camera devices and settings
        framerate=30
        
        # Array of camera topics and capture devices
        self.camera_captures=[]
        self.camera_publishers=[]
        self.cameras=[]

        # Define class stuff
        self.bridge = CvBridge()
        self.pyudev_context = pyudev.Context()

        # Find availible cameras
        self.find_camera_streams()

        # Loop through devices to check if they can be opened and add publishers 
        for device in self.cameras:
            capture = cv2.VideoCapture(device["camera"])
            if not capture.isOpened():
                self.get_logger().error(f'Failed to open {device["camera"]}')
                continue
            
            # Create publisher for each camera stream
            publisher = self.create_publisher(Image,device["topic"],framerate)

            # Log camera streams and publishers
            self.camera_captures.append(capture)
            self.camera_publishers.append(publisher)

        # Timer callback to publish camera feeds 
        self.timer = self.create_timer(1.0/framerate,self.publish_frames)

    def publish_frames(self):
        # Try to publish all camera frames
        for i in range(len(self.camera_captures)):
            # Attempt to capture video frame
            return_value, frame = self.camera_captures[i].read()
            if not return_value: continue

            # If frame capture is successful publish frame
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.camera_publishers[i].publish(msg)

    def find_camera_streams(self):
        i=0
        for device in self.pyudev_context.list_devices(subsystem='video4linux'):
            dev_node = device.device_node

            # Check if device can be opened
            capture = cv2.VideoCapture(dev_node)
            if not capture.isOpened():
                capture.release()
                continue

            # Store camera
            name = device.get('ID_V4L_PRODUCT') or dev_node
            self.cameras.append({"camera": dev_node, "name": name, "topic": f"/camera/rgb/img_raw{i}"})

            capture.release()
            i+=1

# Handle node startup, running, and destorying
def main(args=None):
    rclpy.init(args=args)

    node = SimpleCameraNode()
    rclpy.spin(node)

    for cap in node.camera_captures: cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()