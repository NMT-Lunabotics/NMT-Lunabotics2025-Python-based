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
        self.sequence_thread = None
        self.lock = threading.Lock()
        self.last_msgs = {}

        # Load json file, and index the sequences
        with open(CONFIG_FILE, 'r') as f: self.sequences = json.load(f)

        # Subscriber to trigger automation
        self.create_subscription(Sequence, '/automation_trigger',self.trigger_callback,QoSProfile(depth=0,reliability=ReliabilityPolicy.BEST_EFFORT))

        self.get_logger().info("\033[34mAutomated operations node started.\033[0m")

    # Handle callbacks for requestion that a automation be ran
    def trigger_callback(self, msg):
        # Safty checks
        if msg.name == "interrupt":
            self.get_logger().warn(f"Automated operation interrupted!")
            self.reset_blocking_ids()
            with self.lock:
                self.interrupt = True
                self.is_busy = False
            return
        if not msg.name or (msg.timestamp < self.last_execute): return

        # Load sequence data
        sequence_name = msg.name
        if sequence_name not in self.sequences: return
        sequence = self.sequences[sequence_name]

        with self.lock:
            if self.is_busy: return
            self.is_busy = True
            self.interrupt = False

        # Run sequence in isolated thread so that the thread can be interrupted
        self.sequence_thread = threading.Thread(target=self.run_sequence, args=(sequence,))
        self.sequence_thread.start()

    # Run automation
    def run_sequence(self, sequence):
        for i, step in enumerate(sequence):
            if self.interrupt: break

            # Step function data
            commands_dict = step.get("commands", {})
            duration = step.get("duration", 0.1)

            # Setup new message data which is what will be ran during execution 
            msgs_to_send = {}
            for cmd_type, values in commands_dict.items():
                if cmd_type is not None:
                    msg = Command()
                    msg.blocking_id = self.blocking_id 
                    msg.command = cmd_type
                    msg.data = [float(v) for v in values]
                    msgs_to_send[cmd_type] = msg

            self.last_msgs = msgs_to_send

            # Until function runs to compleation repeatly send commands over serial using inital time logic
            start_time = time.monotonic()
            last_step = (i == len(sequence) - 1)
            sent_last_step_blocking = False

            # Loop through commands and send them for required time period
            while time.monotonic() - start_time < duration:
                if self.interrupt: break
                for cmd_type, msg in msgs_to_send.items():
                    if last_step and not sent_last_step_blocking: msg.blocking_id = -1
                    self.robot_pub.publish(msg)
                # Once finished end command execution
                if last_step and not sent_last_step_blocking: 
                    sent_last_step_blocking = True  
                    self.last_execute=time.time()
                time.sleep(0.05)

            if self.interrupt: break

        with self.lock:
            self.is_busy = False
            self.interrupt = False

        if self.interrupt: self.reset_blocking_ids()
            
    # Send blank command to reset blocking ids of last command message
    def reset_blocking_ids(self):
        for msg in self.last_msgs.values():
            msg.blocking_id = -1
            self.robot_pub.publish(msg)

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