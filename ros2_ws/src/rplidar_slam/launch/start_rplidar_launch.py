from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RPLidar driver
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_driver',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 256000, 'use_sim_time': False}]
        )
    ])
