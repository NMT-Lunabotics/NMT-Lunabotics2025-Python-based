# simple_service.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RobotService(Node):

    def __init__(self):
        super().__init__('robot_service')

        self.srv = self.create_service(Trigger,'robot_command',self.command_callback)

    def command_callback(self, request, response):
        self.get_logger().info("Command received")

        response.success = True
        response.message = "Robot command executed"

        return response


def main():
    rclpy.init()
    node = RobotService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()