from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cameras = [
        {'name': 'camera0', 'video_device': '/dev/video0'},
        # Add more cameras as needed
    ]

    nodes = []
    for camera in cameras:
        nodes.append(
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                namespace=camera['name'],  # Set the namespace for the node
                output='screen'
            )
        )

    return LaunchDescription(nodes)