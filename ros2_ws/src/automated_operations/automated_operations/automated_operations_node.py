#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Command, Sequence
from ament_index_python.packages import get_package_share_directory
import json
import time
import os

# Sequences file path
CONFIG_FILE = os.path.join(get_package_share_directory('automated_operations'),'config','automation_sequences.json')

class AutomationPublisher(Node):
    def __init__(self):
        super().__init__('automated_operations_node')
        self.robot_pub = self.create_publisher(Command, '/robot_commands', 10)
        self.is_busy = False
        self.blocking_id = 1

        # Load json file, and index the sequences
        with open(CONFIG_FILE, 'r') as f: self.sequences = json.load(f)
        self.index_to_sequence = list(self.sequences.keys())

        # Subscriber to trigger automation
        self.create_subscription(Sequence, '/automation_trigger',self.trigger_callback,10)

    def trigger_callback(self, msg):
        # Safty checks
        if self.is_busy or msg.sequence is None: return

        index = msg.sequence
        if index < 0 or index >= len(self.index_to_sequence): return

        # Load sequence data
        sequence_name = self.index_to_sequence[index]
        sequence = self.sequences[sequence_name]

        # Run sequence
        self.is_busy = True
        self.run_sequence(sequence)
        self.is_busy = False

    def run_sequence(self, sequence):
        for step in sequence:
            # Data for step of function
            cmd_type = step.get("command")
            values = step.get("values", [])
            duration = step.get("duration", 0.1)

            # Make new command message
            msg = None
            if cmd_type is not None:
                msg = Command()
                msg.blocking_id = self.blocking_id
                msg.command = cmd_type
                msg.data = [int(v) for v in values]

            # Send command repeatedly for the duration
            start_time = time.monotonic()
            while time.monotonic() - start_time < duration:
                if msg is not None: self.robot_pub.publish(msg)
                time.sleep(0.05) 

def main():
    rclpy.init()
    node = AutomationPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()