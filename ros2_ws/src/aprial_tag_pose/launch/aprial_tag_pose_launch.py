from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aprial_tag_pose',
            executable='aprial_tag_pose_node',
            name='aprial_tag_pose_node'
        )
    ])
