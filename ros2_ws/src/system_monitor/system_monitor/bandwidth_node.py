#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import sys
import threading
from rclpy.qos import QoSProfile
import importlib

class DataUsageMonitor(Node):
    def __init__(self, topics):
        super().__init__('bandwidth_node')
        # variables to hold data
        self.lock = threading.Lock()
        self.topics = topics
        self.stats = {}

        # Loop through all listed topics
        for topic in self.topics:
            topic_name = topic['topic']
            msg_type_name = topic['type']
            module_name, class_name = msg_type_name.rsplit('.', 1)
            module = importlib.import_module(module_name)
            msg_class = getattr(module, class_name)

            # Store data about topic and start topic subscription to monitor the data usage
            self.stats[topic_name] = {'bytes': 0, 'last_time': time.time(), 'rate': 0.0, 'start_time': time.time()}
            self.create_subscription(msg_class, topic_name, self.make_callback(topic_name), QoSProfile(depth=10))

        # Print updated stats every second
        self.create_timer(1.0, self.print_stats)

    def make_callback(self, topic):
        # Create a callback which monitors the data used by a topic over time.
        def callback(msg):
            msg_bytes = sys.getsizeof(msg)
            with self.lock:
                now = time.time()
                dt = now - self.stats[topic]['last_time']
                self.stats[topic]['bytes'] += msg_bytes
                if dt > 0:
                    self.stats[topic]['rate'] = msg_bytes / dt / 1024
                self.stats[topic]['last_time'] = now
        return callback

    def print_stats(self):
        # Ensure threading works correctly
        with self.lock:
            # Print headers
            print(f"{'Topic':<35} {'Total':<12} {'Average':<15} {'Current':<12}")
            print("-"*70)

            for topic, stat in self.stats.items():
                # Calulate usage rate and total rate in MB, KB, B
                total = stat['bytes']
                if total < 1024: 
                    magnetude='B'
                    total_string = f"{total} B"
                elif total < 1024*1024: 
                    magnetude='KB'
                    total_string = f"{total/1024:.2f} KB"
                else: 
                    magnetude='MB'
                    total_string = f"{total/(1024*1024):.2f} MB"

                rate = stat['rate'] * 1024 
                if rate < 1024: rate_string = f"{rate:.0f} B/s"
                elif rate < 1024*1024: rate_string = f"{rate/1024:.2f} KB/s"
                else: rate_string = f"{rate/(1024*1024):.2f} MB/s"

                elapsed=stat['last_time']-stat['start_time']
                if magnetude == "KB": total=total/1024
                elif magnetude == "MB": total=total/(1024*1024)
                average=f"{(total/elapsed):.2f} {magnetude}/s"

                # Print out all bandwidth info in table format
                print(f"{topic:<35} {total_string:<12} {average:<15} {rate_string:<12}")
            print(f"{'-'*70}\n")


def main():
    rclpy.init()
    topics_to_monitor = [
        {'topic': '/map', 'type': 'nav_msgs.msg.OccupancyGrid'},
        {'topic': '/fake_lidar/pose', 'type': 'geometry_msgs.msg.Pose2D'},
        {'topic': "/scan", 'type': "sensor_msgs.msg.LaserScan"}
    ]
    node = DataUsageMonitor(topics_to_monitor)
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
