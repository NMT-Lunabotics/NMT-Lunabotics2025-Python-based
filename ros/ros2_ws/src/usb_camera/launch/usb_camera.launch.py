from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_camera',
            executable='usb_camera_node',  # matches the installed executable
            name='usb_camera_node',
            namespace='camera0',           # optional namespace for multiple cameras
            output='screen',
            parameters=[{
                'video_device': '/dev/video20',  # specify camera device
                'frame_rate': 30.0,
                'pixel_format': 'yuyv',         # or 'mjpeg'
                'camera_name': 'camera0'
            }],
        )
    ])