#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import threading
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

class SLAMMapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer_node')
        self.map_img = None
        self.display_img = None
        self.map_info = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.lock = threading.Lock()

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.create_subscription(Pose2D, '/fake_lidar/pose', self.pose_callback, 50)
        self.create_timer(1.0, self.update_map)      # Map refresh 1 Hz
        self.create_timer(1.0 / 60.0, self.update_pose)  # Pose refresh 60 Hz

    def map_callback(self, msg):
        with self.lock:
            w = msg.info.width
            h = msg.info.height
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            img = np.zeros((h, w), dtype=np.uint8)
            img[data == -1] = 127
            img[data == 0] = 255
            img[data > 0] = 0
            self.map_img = cv2.flip(img, 0)
            self.map_info = msg.info

    def pose_callback(self, msg):
        with self.lock:
            self.robot_x = msg.x
            self.robot_y = msg.y
            self.robot_theta = msg.theta

    def update_map(self):
        with self.lock:
            if self.map_img is None:
                return
            self.display_img = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)

    def update_pose(self):
        with self.lock:
            if self.display_img is None or self.map_info is None:
                return
            img = self.display_img.copy()

            mx = int((self.robot_x - self.map_info.origin.position.x) / self.map_info.resolution)
            my = int((self.robot_y - self.map_info.origin.position.y) / self.map_info.resolution)
            my = img.shape[0] - my

            # Draw robot center
            cv2.circle(img, (mx, my), 2, (0, 0, 255), -1)

            # Draw heading line
            length = 5  # pixels
            end_x = int(mx + length * np.cos(self.robot_theta))
            end_y = int(my - length * np.sin(self.robot_theta))  # subtract because image y is down
            cv2.line(img, (mx, my), (end_x, end_y), (0, 255, 0), 1)

            self.display_img = img

def main():
    rclpy.init()
    node = SLAMMapViewer()
    cv2.namedWindow("SLAM Map", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("SLAM Map", 800, 800)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.display_img is not None:
                cv2.imshow("SLAM Map", node.display_img)
                cv2.waitKey(1)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
