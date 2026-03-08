#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Command
import time
import yaml
import os
import serial
import serial.tools.list_ports

class serialCommands:
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
        self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
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

    def send_serial(self, message: str)->None:
        """Send a string to the Arduino"""
        self.ser.write((message + "\n").encode())
        self.ser.flush()

    def close_serial(self)->None:
        """Close the serial connection"""
        if self.ser:
            self.ser.close()
            self.ser = None

    def read_raw_serial(self)->str:
        """Reads the raw audio byte stream from Arduino"""
        return self.ser.read(self.ser.in_waiting or 1)
    
    def read_serial(self)->list:
        """Read all available lines from Arduino and return as a list"""
        lines = []
        while self.ser.in_waiting > 0:
            line = self.ser.readline()
            if line:
                lines.append(line.decode(errors='ignore').strip())
        return lines if lines else None
    
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
        self.ser.write(msg)
        self.ser.flush()
        time.sleep(0.001)

    def read_command_feedback(self)->list:
        """Read feedback from command operations"""
        packets = []
        expected_length = None
        while self.ser.in_waiting > 0:
            b = self.ser.read(1)[0]
            self._read_buffer.append(b)
            buffer = self._read_buffer

            if len(buffer) == 1 and buffer[0] != self.startByte:
                self._read_buffer.clear()
                continue
            if len(buffer) == 2:
                expected_length = buffer[1]
                continue
            if expected_length is not None and len(buffer) >= expected_length + 3:
                if buffer[expected_length + 2] == self.endbyte:
                    command = chr(buffer[2])
                    data = list(buffer[3:3 + expected_length - 1])
                    packets.append({"command": command, "data": data})
                    self._read_buffer = self._read_buffer[expected_length + 3:]
                else:
                    self._read_buffer = self._read_buffer[1:]
        return packets if packets else None

class SerialCommandNode(Node):
    def __init__(self):
        super().__init__('serial_writer_node')

        try:
            self.ser = serialCommands()  # could throw RuntimeError
        except RuntimeError as e:
            self.get_logger().error(f"Serial initialization failed: {e}")
            # optionally exit cleanly
            rclpy.shutdown()

        # Subscribe to the topic
        self.sub = self.create_subscription(Command,'/serial/writer',self.handle_command,10)

    def handle_command(self, msg):
        combined = [int(d) for d in msg.data]    
        self.ser.send_command(msg.command, combined)

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()