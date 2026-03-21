from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='showimage',
            name='showimage',
            arguments=['--ros-args', '--remap', 'image:=/camera/stream'],  
            output='screen',
            additional_env={'ROS_DOMAIN_ID': '11'}
        )
    ])
