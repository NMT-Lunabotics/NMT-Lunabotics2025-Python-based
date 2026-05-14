#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import threading
from rclpy.qos import QoSProfile
import importlib
from rclpy.serialization import serialize_message

class DataUsageMonitor(Node):
    def __init__(self, topics):
        super().__init__('bandwidth_node')

        self.lock = threading.Lock()
        self.topics = topics
        self.stats = {}

        # Subscribe to each topic dynamically
        for topic in self.topics:
            topic_name = topic['topic']
            msg_type_name = topic['type']
            module_name, class_name = msg_type_name.rsplit('.', 1)
            module = importlib.import_module(module_name)
            msg_class = getattr(module, class_name)

            # Initialize stats
            now = time.time()
            self.stats[topic_name] = {
                'bytes_total': 0,
                'last_time': now,
                'current_rate': 0.0,
                'start_time': now
            }

            # Create subscription with a callback
            self.create_subscription(
                msg_class,
                topic_name,
                self.make_callback(topic_name),
                QoSProfile(depth=10)
            )

        # Print updated stats every second
        self.create_timer(1.0, self.print_stats)

    def make_callback(self, topic_name):
        def callback(msg):
            # Use ROS 2 serialization for accurate size
            msg_bytes = len(serialize_message(msg))
            now = time.time()
            with self.lock:
                dt = now - self.stats[topic_name]['last_time']
                self.stats[topic_name]['bytes_total'] += msg_bytes
                self.stats[topic_name]['current_rate'] = msg_bytes / dt if dt > 0 else 0.0
                self.stats[topic_name]['last_time'] = now
        return callback

    def print_stats(self):
        with self.lock:
            print(f"{'Topic':<35} {'Total':<15} {'Average':<15} {'Current':<12}")
            print("-"*80)
            for topic, stat in self.stats.items():
                total_bytes = stat['bytes_total']
                elapsed = stat['last_time'] - stat['start_time']
                avg_rate = total_bytes / elapsed if elapsed > 0 else 0.0

                # Format total size
                if total_bytes < 1024:
                    total_str = f"{total_bytes} B"
                elif total_bytes < 1024*1024:
                    total_str = f"{total_bytes/1024:.2f} KB"
                else:
                    total_str = f"{total_bytes/(1024*1024):.2f} MB"

                # Format average rate
                if avg_rate < 1024:
                    avg_str = f"{avg_rate:.0f} B/s"
                elif avg_rate < 1024*1024:
                    avg_str = f"{avg_rate/1024:.2f} KB/s"
                else:
                    avg_str = f"{avg_rate/(1024*1024):.2f} MB/s"

                # Format current rate
                cur_rate = stat['current_rate']
                if cur_rate < 1024:
                    cur_str = f"{cur_rate:.0f} B/s"
                elif cur_rate < 1024*1024:
                    cur_str = f"{cur_rate/1024:.2f} KB/s"
                else:
                    cur_str = f"{cur_rate/(1024*1024):.2f} MB/s"

                print(f"{topic:<35} {total_str:<15} {avg_str:<15} {cur_str:<12}")
            print("-"*80 + "\n")


def main():
    rclpy.init()
    topics_to_monitor = [
        {'topic': "/joy", 'type': "sensor_msgs.msg.Joy"},
        {'topic': "/actuators", 'type': "controller_input.msg.Actuators"},
        {'topic': "/camera/toggle_view", 'type': "controller_input.msg.Camera"},
        {'topic': "/cmd_vel", 'type': "geometry_msgs.msg.Twist"}
    ]
    node = DataUsageMonitor(topics_to_monitor)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
