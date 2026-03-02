from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_command',
            executable='serial_writer_node',
            name='serial_writer_node',
            additional_env={'ROS_DOMAIN_ID': '10'}
        )
    ])
