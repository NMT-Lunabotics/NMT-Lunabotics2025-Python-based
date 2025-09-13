from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='pi_camera_node',
            output='screen',
            parameters=[{
                'image_width': 640,
                'image_height': 480,
                'frame_rate': 30,
                'device': '/dev/video0',
            }],
            remappings=[('/image_raw', '/camera/image_raw')]
        ),
    ])
