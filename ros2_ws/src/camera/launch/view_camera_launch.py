from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        Node(
            # Custom camera view to handle window size
            package='camera',
            executable='camera_view_node.py',
            name='camera_view',
            parameters=[{
                'image_topic': '/camera/stream',
                'window_width': 1024,
                'window_height': 768,
                'fullscreen': False
            }],
            output='screen'
        )
    ])
