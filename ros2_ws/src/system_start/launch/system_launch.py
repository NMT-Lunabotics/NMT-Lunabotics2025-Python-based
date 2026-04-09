from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launches the robot systems (opens camera, starts apriltag positioning, starts serial commuications)"""
    # Get file paths of diffrent packages
    camera_directory=get_package_share_directory('camera')
    system_start_directory=get_package_share_directory('system_start')

    # Get file paths of needed files from directorys
    camera_launch_file=os.path.join(camera_directory,'launch','usb_camera_launch.py')
    camera_config_file=os.path.join(system_start_directory,'config','cameras.yaml')

    # Setup system launchers
    camera_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={'camera_config': camera_config_file}.items()
    )
    apriltag_node=Node(package='camera',executable='camera_apriltag_node.py',name='camera_apriltag_node')
    serial_talker_node=Node(package='serial_commands',executable='serial_talker_node.py',name='serial_talker_node')
    automated_operations=Node(package='automated_operations',executable='automated_operations_node.py',name='automated_operations_node')

    # Launch all specifyied launch files and nodes
    return LaunchDescription([automated_operations,camera_launch,apriltag_node, serial_talker_node])