from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_commands',
            executable='serial_talker_node.py',
            name='serial_talker_node'
        )
    ])