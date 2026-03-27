from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    camera_dir = get_package_share_directory('camera')
    controller_dir = get_package_share_directory('controller_input')

    # Get launch file paths
    camera_launch_file = os.path.join(camera_dir, 'launch', 'view_camera_launch.py')
    controller_launch_file = os.path.join(controller_dir, 'launch', 'teleop_launch.py')

    # Include launch files
    camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch_file))
    controller_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch_file))

    # Launch everything
    return LaunchDescription([controller_launch,camera_launch])