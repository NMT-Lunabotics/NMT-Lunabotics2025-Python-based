from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_cfg = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'slam_toolbox.yaml'
    )

    slam_pkg = get_package_share_directory('slam_toolbox')
    slam_launch = os.path.join(slam_pkg, 'launch', 'online_async_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'params_file': bringup_cfg}.items()
        )
    ])

