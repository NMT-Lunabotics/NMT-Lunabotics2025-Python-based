#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class FastCropNode(Node):
    def __init__(self):
        super().__init__('fast_crop_node')
        self.bridge = CvBridge()

        self.rgb_pub = self.create_publisher(Image, '/camera/camera/color/image_raw_cropped', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/camera/aligned_depth_to_color/image_raw_cropped', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info_cropped', 10)

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)

        self.x = 40
        self.y = 0
        self.w = 560
        self.h = 480

        self.latest_info = None

        self.get_logger().info("FastCropNode started")

    def info_cb(self, msg):
        self.latest_info = msg

    def rgb_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        cropped = img[self.y:self.y + self.h, self.x:self.x + self.w]

        out = self.bridge.cv2_to_imgmsg(cropped, msg.encoding)
        out.header = msg.header
        self.rgb_pub.publish(out)

        self.publish_info(msg.header)

    def depth_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        cropped = img[self.y:self.y + self.h, self.x:self.x + self.w]

        out = self.bridge.cv2_to_imgmsg(cropped, msg.encoding)
        out.header = msg.header
        self.depth_pub.publish(out)

    def publish_info(self, header):
        if self.latest_info is None:
            return

        info = CameraInfo()
        info.header = header

        info.width = self.w
        info.height = self.h

        info.k = list(self.latest_info.k)
        info.p = list(self.latest_info.p)
        info.d = list(self.latest_info.d)
        info.distortion_model = self.latest_info.distortion_model

        info.k[2] -= self.x  
        info.k[5] -= self.y 
        info.p[2] -= self.x
        info.p[6] -= self.y

        self.info_pub.publish(info)


def main():
    rclpy.init()
    rclpy.spin(FastCropNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()