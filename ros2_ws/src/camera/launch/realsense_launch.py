from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',

            'enable_color': 'true',
            'enable_depth': 'true',

            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',

            'align_depth.enable': 'true',

            'enable_gyro': 'true',
            'enable_accel': 'true',

            'pointcloud.enable': 'true',
        }.items()
    )

    return LaunchDescription([
        realsense_launch
    ])