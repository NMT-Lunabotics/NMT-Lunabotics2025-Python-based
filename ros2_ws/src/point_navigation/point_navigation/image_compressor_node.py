#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor_node')
        self.bridge=CvBridge()
        self.pub=self.create_publisher(CompressedImage,'/camera/camera/color/image_raw/compressed2',10)
        self.create_subscription(Image,'/camera/camera/color/image_raw',self.cb,10)
        self.get_logger().info("ImageCompressor started")

    def cb(self,msg):
        img=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        out=self.bridge.cv2_to_compressed_imgmsg(img,dst_format='jpeg')
        out.header=msg.header
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ImageCompressor())
    rclpy.shutdown()

if __name__=='__main__':
    main()