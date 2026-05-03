#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from robot_interfaces.msg import Camera

class CameraMux(Node):
    def __init__(self):
        super().__init__('camera_mux_node')
        # Cameras to switch between
        self.cameras = {
            0: 'camera0/image_raw/compressed',
            1: 'camera1/image_raw/compressed',
            2: 'camera2/image_raw/compressed',
        }
        self.camera_indexes = [0,1]
        self.index_cycle_state=0

        self.active_index = 0
        self.last_index = None
        
        # Create publisher settings, keep only newest images
        publisher_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,history=QoSHistoryPolicy.KEEP_LAST,depth=1,durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(CompressedImage, '/camera/stream', publisher_qos)
        self.latest_frames = {}

        self.declare_parameter('output_fps', 5.0)
        self.output_fps = self.get_parameter('output_fps').value
        
        # Track time between publishs to force a framerate since cameras only can be set so low
        self.last_publish_time = self.get_clock().now()
        self.frames_received = 0
        self.frames_published = 0

        # Subscribe to each of the camera topics
        self.subs = []
        for idx, topic in self.cameras.items():
            sub_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1)
            def make_callback(index): return lambda msg: self.image_active(msg, index)
            self.subs.append(self.create_subscription(CompressedImage, topic, make_callback(idx), sub_qos))

        # Setup system timers and toggle topic
        self.create_subscription(Camera, '/camera/toggle_view', self.toggle_stream, 10)
        self.timer = self.create_timer(1.0 / self.output_fps, self.publish_active)

        self.get_logger().info("\033[34mCamera mux node started.\033[0m")

    # Switch camera streams
    def toggle_stream(self, msg: Camera):
        if msg.camera_increment == True:
            if self.index_cycle_state == 0:
                self.index_cycle_state = 1
                index = self.camera_indexes[self.active_index] + 1
                if index >= len(self.cameras):index = 0
                if index == self.camera_indexes[1 - self.active_index]:
                    index += 1
                    if index >= len(self.cameras): index = 0
                self.camera_indexes[self.active_index] = index
        else: self.index_cycle_state=0

        if msg.camera_view in self.cameras:
            self.active_index = msg.camera_view
            self.last_index = self.active_index

    # Publish active camera stream to the camera stream topic, only keep last frame
    def image_active(self, msg, index):
        self.frames_received += 1
        self.latest_frames[index] = msg

    # Keep track of number of frames and ensure that topic is frame limited
    def publish_active(self):
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_publish_time).nanoseconds / 1e9
        
        # Rate limit the topic
        if time_since_last >= (1.0 / self.output_fps) and self.active_index in self.latest_frames:
            self.pub.publish(self.latest_frames[self.camera_indexes[self.active_index]])
            self.frames_published += 1
            self.last_publish_time = current_time

def main():
    rclpy.init()
    node = CameraMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()