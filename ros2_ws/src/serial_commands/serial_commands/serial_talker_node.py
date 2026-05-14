#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_interfaces.msg import Command
import time
import yaml
import os
import serial
import serial.tools.list_ports

# Converts /cmd_vel and /actuator messages which range from [-1,1] to [-30,30] or [-25,25] which are sent to run robot 

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
        self.blocking={}
        self.blocking_timer={}

        self.blocking_timeout=3
        self.blocking_last_time=time.monotonic()

        self.actuator_vel_scale=25  # Actuator velocity scale (-25 - 25)
        self.motor_vel_scale=30     # Motor velocity scale (-30 - 30)
        self.wheel_base=2           # Distance between wheels (m)

        try:
            self.serial = serialCommands()  # could throw RuntimeError
            self.get_logger().info(f"Arduino started on port: {self.serial.port}")
        except RuntimeError as e:
            self.get_logger().error(f"Serial initialization failed: {e}")
            # optionally exit cleanly
            rclpy.shutdown()

        # Subscribe topics that include robot command data
        self.cmd_vel = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.cmd = self.create_subscription(Command,'/robot_commands',self.cmd_callback,10)

        self.get_logger().info("\033[34mSerial talker node started.\033[0m")

    # System callbacks which handle the the cmd_vel and /robot_command topics
    def cmd_callback(self, msg):
        self.handle_special_cmd(msg.command, msg.data, msg.blocking_id)
    def cmd_vel_callback(self, msg):
        self.send_vel_command([msg.linear.x,msg.angular.z],None,vel_source=True)

    # Handle speical cmd cases, right now only the 'M' and 'A' speical cases exist
    def handle_special_cmd(self, command, data, blocking_id):
        if command == "M": self.send_vel_command(data, blocking_id,vel_source=False)
        elif command == "A": self.send_act_command(data, blocking_id)
        else: self.handle_command(command, data, blocking_id)
    
    # Handles the sent command taking into account if a command is blocking
    def handle_command(self, command, data, blocking_id):
        if not command: return
        #self.get_logger().info(f"Sending command {command} with data {data}")
        # Blocking and command sender logic
        if command in self.blocking:
            if blocking_id == -1:
                del self.blocking[command]
                del self.blocking_timer[command]
            elif blocking_id == self.blocking[command]:
                self.blocking_timer[command] = time.monotonic()
                combined = [int(d) for d in data]    
                self.serial.send_command(command, combined)
            elif time.monotonic() - self.blocking_timer[command] >= self.blocking_timeout:
                del self.blocking[command]
                del self.blocking_timer[command]
        else:
            # Only add blocking if blocking_id is set and not 0
            if blocking_id not in (-1, None, 0): 
                self.blocking[command] = blocking_id
                self.blocking_timer[command] = time.monotonic()

            combined = [int(d) for d in data]    
            self.serial.send_command(command, combined)

    # Apply diffrential driving to 'M' command
    def send_vel_command(self, data, blocking_id, vel_source):
        if vel_source:
            velocity=data[0]
            angular_velocity=data[1]

            # Scale motor command
            left_motor=velocity-(angular_velocity*self.wheel_base/2)
            right_motor=velocity+(angular_velocity*self.wheel_base/2)

            max_val = max(abs(left_motor), abs(right_motor), 1.0)

            left_motor = int((left_motor/max_val)*self.motor_vel_scale)
            right_motor = int((right_motor/max_val)*self.motor_vel_scale)

            self.handle_command('M', [left_motor, right_motor], blocking_id)
        else:
            left_input = data[0]
            right_input = data[1]

            # Apply differential steering: compute linear and angular components
            linear = (right_input + left_input) / 2
            angular = (right_input - left_input) / self.wheel_base

            # Compute motor outputs using differential drive
            left_motor = linear - (angular * self.wheel_base / 2)
            right_motor = linear + (angular * self.wheel_base / 2)

            # Scale to actual motor range
            max_val = max(abs(left_motor), abs(right_motor), 1)  # avoid div by zero
            left_motor = int((left_motor / max_val) * self.motor_vel_scale)
            right_motor = int((right_motor / max_val) * self.motor_vel_scale)

            # Send to motors
            self.handle_command('M', [left_motor, right_motor], blocking_id)

    # Apply scaling to 'A' command
    def send_act_command(self, data, blocking_id):
        arm_act_max_pos=data[0]
        arm_act_min_pos=data[1]
        bucket_act_max_pos=data[2]
        bucket_act_min_pos=data[3]
        arm_velocity=data[4]
        bucket_velocity=data[5]

        if arm_act_max_pos != -1 and arm_act_min_pos != -1: 
            arm_velocity=1
        else: 
            # Scale speeds to actuator speed scale
            arm_velocity = int(arm_velocity*self.actuator_vel_scale)
            # Disable absolute positioning
            arm_act_max_pos=-1
            arm_act_min_pos=-1

        if bucket_act_max_pos != -1 and bucket_act_min_pos != -1:
            bucket_velocity=1
        else:
            # Scale speeds to actuator speed scale
            bucket_velocity = int(bucket_velocity*self.actuator_vel_scale)
            # Disable absolute positioning
            bucket_act_max_pos=-1
            bucket_act_min_pos=-1

        self.handle_command('A', [arm_act_max_pos, arm_act_min_pos, bucket_act_max_pos, bucket_act_min_pos, arm_velocity, bucket_velocity], blocking_id)

def main(args=None):
    rclpy.init(args=args)
    node = SerialTalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()