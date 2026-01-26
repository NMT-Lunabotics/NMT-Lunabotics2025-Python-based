#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from serial_command.msg import Command
from aprial_tag_pose.msg import Pose

class PathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner_node')

        # Create topic publisher to drive robot, and timer to execute command on loop
        self.pub = self.create_publisher(Command,'/serial/writer',10)
        self.aprial_tag=self.create_subscription(Pose,'/aprial_tag/pose',self.stop,1)
        self.timer = self.create_timer(0.1, self.send_forward)
        self.stop=False

    def send_forward(self):
        if self.stop: return
        msg = Command()
        msg.command = 'M'
        msg.data = [5,5]               

        self.pub.publish(msg)
        self.get_logger().debug('Sent forward command')

    def stop(self, msg):
        if(msg.distance < 3 and msg.id!=-1):
            self.stop=True
        else: self.stop=False

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()

    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
