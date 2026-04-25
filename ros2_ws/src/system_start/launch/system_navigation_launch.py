from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('camera'),
                'launch',
                'realsense_launch.py'
            )
        )
    )

    apriltag_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='apriltag_world_frame',
        arguments=['0','0','0','0','0','0','map','apriltag']
    )

    apriltag_node = Node(
        package='camera',
        executable='camera_apriltag_node.py',
        name='camera_apriltag_node',
        parameters=[{
            'camera_topic': '/camera/camera/color/image_raw',
            'publish_map': True,
            'publish_processed': True
        }]
    )

    base_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('point_navigation'),
                'launch',
                'realsense_slam.py'
            )
        )
    )

    return LaunchDescription([
        realsense_launch,
        apriltag_frame,
        apriltag_node,
        base_frame,
        rtabmap_launch
    ])