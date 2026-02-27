#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from serial_commands.msg import Command
import serial
import serial.tools.list_ports
import time
import os
import yaml
import string


class SerialListener:
    def __init__(self, port=None, baudrate=115200, timeout=0.1):
        self.startByte = 2
        self.endByte = 3
        self._read_buffer = bytearray()

        self.port = port or self.find_arduino()
        self.ser = serial.Serial(self.port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)

    def find_arduino(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or port.vid is not None:
                return port.device
        raise RuntimeError("Arduino not found")

    def read_command_feedback(self):
        """Read all available feedback packets from Arduino and split into numbers and text."""
        packets = []

        while self.ser.in_waiting > 0:
            b = self.ser.read(1)
            if not b:
                break
            self._read_buffer += b

            while True:
                if len(self._read_buffer) < 3:
                    break  # not enough bytes yet

                if self._read_buffer[0] != self.startByte:
                    self._read_buffer = self._read_buffer[1:]
                    continue

                expected_length = self._read_buffer[1]
                if len(self._read_buffer) < expected_length + 3:
                    break  # full packet not yet received

                if self._read_buffer[expected_length + 2] != self.endByte:
                    # bad packet, skip first byte
                    self._read_buffer = self._read_buffer[1:]
                    continue

                # extract packet
                command = chr(self._read_buffer[2])
                data_bytes = self._read_buffer[3:3 + expected_length - 1]

                # Split into numbers and text
                data_list = []
                text_chars = []
                for byte in data_bytes:
                    char = chr(byte)
                    if char in string.printable and not char.isspace():
                        text_chars.append(char)
                    else:
                        data_list.append(byte)

                packets.append({
                    'command': command,
                    'data': data_list,
                    'text': ''.join(text_chars)
                })

                # Remove processed packet from buffer
                self._read_buffer = self._read_buffer[expected_length + 3:]

        return packets


class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        # Load serial config
        config_path = os.path.join(os.path.dirname(__file__), '../config/serial_params.yaml')
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                serial_conf = yaml.safe_load(f)
        else:
            serial_conf = {}

        port = serial_conf.get('port', None)
        baudrate = serial_conf.get('baudrate', 115200)
        timeout = serial_conf.get('timeout', 0.1)

        try:
            self.ser = SerialListener(port=port, baudrate=baudrate, timeout=timeout)
        except RuntimeError as e:
            self.get_logger().error(f"Serial initialization failed: {e}")
            rclpy.shutdown()
            return

        # Publisher for decoded messages
        self.pub = self.create_publisher(Command, '/serial/listener', 10)

        # Timer to poll serial periodically (100 Hz)
        self.timer = self.create_timer(0.01, self.poll_serial)

    def poll_serial(self):
        packets = self.ser.read_command_feedback()
        for pkt in packets:
            msg = Command()
            msg.command = pkt['command']
            msg.data = pkt['data']  # numeric bytes
            msg.text = pkt['text']  # printable text
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
