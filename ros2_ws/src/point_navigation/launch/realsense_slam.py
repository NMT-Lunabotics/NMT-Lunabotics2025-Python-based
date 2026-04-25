from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            )
        ),
        launch_arguments={

            # REQUIRED topics
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',

            # SLAM config (matching your CLI)
            'approx_sync': 'true',
            'rgbd_odometry': 'true',
            'Odom/Type': 'RGBD/IMU',
            'Odom/IMU/Topic': '/imu/data',
            'Odom/IMU/FrameId': 'camera_link',

            'Rtabmap/DetectionRate': '10',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',

            'Rtabmap/PublishCloud': 'false',
            'Rtabmap/CloudVoxelSize': '0.05',
            'Rtabmap/PublishGridMap': 'false',

            'RGBD/DepthDecimation': '1',
            'delete_db_on_start': 'true',

            'Grid/FromDepth': 'true',
            'Grid/RangeMax': '5.0',
            'Grid/CellSize': '0.1',
            'Grid/3D': 'true',

            'Mem/IncrementalMemory': 'true',
        }.items()
    )

    return LaunchDescription([
        rtabmap_launch
    ])