#!/usr/bin/env python3
import rclpy, time, json, os, threading
from rclpy.node import Node
from robot_interfaces.msg import Command, Sequence
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Sequences file path
CONFIG_FILE = os.path.join(get_package_share_directory('automated_operations'),'config','automation_sequences.json')

class AutomationPublisher(Node):
    def __init__(self):
        super().__init__('automated_operations_node')
        self.robot_pub = self.create_publisher(Command, '/robot_commands', QoSProfile(depth=10))
        self.is_busy = False
        self.blocking_id = 1
        self.last_execute=time.time()
        self.interrupt = False

        # Load json file, and index the sequences
        with open(CONFIG_FILE, 'r') as f: self.sequences = json.load(f)

        # Subscriber to trigger automation
        self.create_subscription(Sequence, '/automation_trigger',self.trigger_callback,QoSProfile(depth=0,reliability=ReliabilityPolicy.BEST_EFFORT))

        self.get_logger().info("\033[34mAutomated operations node started.\033[0m")

    def trigger_callback(self, msg):
        # Safty checks
        if msg.name == "interrupt":
            self.get_logger().error(f"LOGGER FOUND INTERRUPT")
            self.interrupt = True
            return
        if self.is_busy or not msg.name or (msg.timestamp < self.last_execute): return

        # Load sequence data
        sequence_name = msg.name
        if sequence_name not in self.sequences: return
        sequence = self.sequences[sequence_name]

        # Run sequence
        self.is_busy = True
        self.interrupt = False
        self.run_sequence(sequence)
        self.is_busy = False

    def run_sequence(self, sequence):
        for i, step in enumerate(sequence):
            if self.interrupt: return
            # Data for step of function
            commands_dict = step.get("commands", {})
            duration = step.get("duration", 0.1)

            # Make new command messages
            msgs_to_send = {}
            for cmd_type, values in commands_dict.items():
                if cmd_type is not None:
                    msg = Command()
                    msg.blocking_id = self.blocking_id  # default blocking_id
                    msg.command = cmd_type
                    msg.data = [float(v) for v in values]
                    msgs_to_send[cmd_type] = msg

            # Send commands repeatedly for the duration
            start_time = time.monotonic()
            last_step = (i == len(sequence) - 1)
            sent_last_step_blocking = False

            while time.monotonic() - start_time < duration:
                if self.interrupt: return
                for cmd_type, msg in msgs_to_send.items():
                    if last_step and not sent_last_step_blocking: msg.blocking_id = -1
                    self.robot_pub.publish(msg)

                if last_step and not sent_last_step_blocking: 
                    sent_last_step_blocking = True  
                    self.last_execute=time.time()
                rclpy.spin_once(self, timeout_sec=0.05)

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