#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from controller_input.msg import Actuators
import time
import yaml
import os
import serial
import serial.tools.list_ports

class serialCommands:
    """Serial class, used to communicate with the arduino controller"""
    def __init__(self)->None:
        """Default function variables"""
        # Open serial config file and read arduino settings
        
        config_path = os.path.join(os.path.dirname(__file__), '../config/serial_params.yaml')

        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                serial_conf = yaml.safe_load(f)
        else:
            serial_conf = {}

        # Read serial settings
        port = serial_conf.get('port', None)
        baudrate = serial_conf.get('baudrate', 115200)
        timeout = serial_conf.get('timeout', 0.1)

        # Initialize Arduino
        self.baudrate = baudrate
        self.port = port or self.find_arduino()
        self.timeout=timeout
        self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        self.startByte=2
        self.endbyte=3
        self._read_buffer = bytearray()
        time.sleep(2)

    def find_arduino(self)->str:
        """Auto-detect Arduino port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or port.vid is not None:
                return port.device
        raise RuntimeError("Arduino not found")

    def close_serial(self)->None:
        """Close the serial connection"""
        if self.serial:
            self.serial.close()
            self.serial = None
    
    def send_command(self, command: str, data: list)->None:
        """Send required data string command to arduino over serial."""
        length = len(data)+1
        msg = bytearray()
        msg.append(self.startByte)
        msg.append(length)
        msg.append(ord(command))
        for d in data:
            msg.append(d % 256)
        msg.append(self.endbyte)
        self.serial.write(msg)
        self.serial.flush()
        time.sleep(0.001)

class SerialTalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')

        self.actuator_max_vel=25
        self.actuator_min_vel=0.5

        self.actuator_vel_scale=25
        self.motor_vel_scale=30

        try:
            self.serial = serialCommands()  # could throw RuntimeError
            self.get_logger().info(f"Arduino started on port: {self.serial.port}")
        except RuntimeError as e:
            self.get_logger().error(f"Serial initialization failed: {e}")
            # optionally exit cleanly
            rclpy.shutdown()

        # Subscribe topics that include robot command data
        self.cmd_vel = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.actuators = self.create_subscription(Twist,'/actuators',self.actuators_callback,10)
    
    def handle_command(self, msg):
        combined = [int(d) for d in msg.data]    
        self.serial.send_command(msg.command, combined)

    def cmd_vel_callback(self, msg):
        # Get data from velocity topic
        velocity=msg.linear.x
        angular_velocity=msg.angular.z

        # Apply diffrential driving and send motor command
        left_motor = int(velocity - angular_velocity)*self.motor_vel_scale
        right_motor = int(velocity + angular_velocity)*self.motor_vel_scale
        self.serial.send_command('M', [left_motor, right_motor])

    def actuators_callback(self, msg):
        ...

def main(args=None):
    rclpy.init(args=args)
    node = SerialTalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()