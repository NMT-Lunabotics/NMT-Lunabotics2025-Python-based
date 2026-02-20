from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            arguments=['image:=/camera/stream'],  
            output='screen'
        )
    ])
