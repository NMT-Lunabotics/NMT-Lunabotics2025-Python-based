from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to launch files
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_slam'),
        'launch',
        'start_rplidar_launch.py'
    )

    # Reference URDF directly from src folder
    urdf_file = os.path.join(
        get_package_share_directory('rplidar_slam'),
        'urdf',
        'goliath.urdf'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # RPLIDAR driver launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file),
    )

    # Static transforms
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
    )

    odom_to_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # SLAM Toolbox node (live mapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[('scan', '/scan')],
        parameters=[{
            'use_sim_time': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'use_odometry': False,
            'provide_odom_frame': False,
            'mode': 'mapping',               # <-- must be 'mapping' for live map
            'map_update_interval': 0.5,
            'scan_queue_size': 100,
            'tf_buffer_duration': 30.0,
            'publish_period_sec': 0.05,
            'scan_downsample': 1,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.0,
            'minimum_travel_heading': 0.0,
            'approx_sync': True,
            'debug_logging': True,
            'min_laser_range': 0.2,
            'max_laser_range': 12.0,
            'transform_publish_period': 0.02,
            'throttle_scans': 1,
            'transform_timeout': 0.2,
            'always_send_full_map': True,
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rplidar_launch,
        laser_tf_node,
        odom_to_base_node,
        slam_toolbox_node,
    ])
