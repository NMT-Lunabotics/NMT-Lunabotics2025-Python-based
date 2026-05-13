#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf2_ros
import math

class SimpleFollower(Node):
    def __init__(self):
        super().__init__('point_navigation')

        self.sub = self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.plan = None
        self.timer = self.create_timer(0.1, self.loop)

    def plan_cb(self, msg):
        self.plan = msg

    def loop(self):
        if self.plan is None or len(self.plan.poses) < 5:
            return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except:
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        q = trans.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        p = self.plan.poses[min(8, len(self.plan.poses) - 1)].pose.position

        dx = p.x - x
        dy = p.y - y

        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)

        angle_error = target_angle - yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # raw commands
        linear = 0.4 * distance
        angular = 1.2 * angle_error

        # HARD CAPS (always enforced)
        linear = max(-0.2, min(0.2, linear))
        angular = max(-0.2, min(0.2, angular))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        # optional stop
        if distance < 0.1:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = SimpleFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()