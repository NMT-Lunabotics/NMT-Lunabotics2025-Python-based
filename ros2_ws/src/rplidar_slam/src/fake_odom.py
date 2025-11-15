#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
import math

# Euler to quaternion helper
def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return (qx, qy, qz, qw)

class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot_slam')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Timers
        self.create_timer(0.1, self.publish_odom)  # 10 Hz
        self.create_timer(0.1, self.publish_scan)  # 10 Hz

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Motion parameters
        self.v = 0.05   # m per cycle
        self.omega = 0.02 # rad per cycle

        # Laser parameters
        self.angle_min = -math.pi/4
        self.angle_max = math.pi/4
        self.angle_increment = math.pi/180
        self.range_min = 0.2
        self.range_max = 5.0

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Move robot in circle
        self.theta += self.omega
        self.x += self.v * math.cos(self.theta)
        self.y += self.v * math.sin(self.theta)

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(msg)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        n = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        ranges = []

        # Simulate a wall at x=2.0 in world frame
        for i in range(n):
            angle = msg.angle_min + i * msg.angle_increment
            # Laser beam in world coordinates
            lx = self.x + math.cos(self.theta + angle) * 2.0
            ly = self.y + math.sin(self.theta + angle) * 2.0
            r = math.hypot(lx - self.x, ly - self.y)
            r = max(msg.range_min, min(msg.range_max, r))
            ranges.append(r)

        msg.ranges = ranges
        self.scan_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
