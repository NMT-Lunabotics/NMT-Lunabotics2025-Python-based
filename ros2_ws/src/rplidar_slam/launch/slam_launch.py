from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_slam'),
        'launch',
        'start_rplidar_launch.py'
    )
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )
    slam_params_file = os.path.join(
        get_package_share_directory('rplidar_slam'),
        'config',
        'slam_params.yaml'
    )

    return LaunchDescription([
        # Launch rplidar driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        ),

        # Start listener / processing node
        #Node(
        #    package='rplidar_slam',
        #    executable='laser_scan_node.py',
        #    output='screen'
        #),

        # Set static transforms
        Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            output='screen',
            parameters=[{
                'publish_odom': 'odom',
                'publish_tf': True,
                'max_iterations': 20
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0','0','0','0','0','0','odom','base_link']
        ),
        # Launch SLAM Toolbox launch
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[('scan', '/scan')],
            parameters=[{
                'use_sim_time': False,           # Only true if using simulation
                'map_frame': 'map',              # The map frame
                'odom_frame': 'odom',            # Odometry frame (not used here)
                'base_frame': 'base_link',       # Robot base frame
                'scan_topic': '/scan',           # If you use your trimmed scan, else /scan
                'use_odometry': False,            # No wheel odometry available
                'provide_odom_frame': False,      # Still publish odom frame
                'mode': 'mapping',               # SLAM mode (not localization)
                'map_update_interval': 0.5,      # Map update frequency in seconds
                'scan_queue_size': 100,          # Size of incoming scan queue
                'tf_buffer_duration': 30.0,      # TF history buffer in seconds
                'publish_period_sec': 0.05,      # How often to publish map updates
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_travel_distance': 0.0,
                'minimum_travel_heading': 0.0,
                'debug_logging': True,
                'min_laser_range': 0.2,
                'max_laser_range': 12.0,
                'scan_downsample': 575,
                'transform_publish_period': 0.02,
                'throttle_scans': 1,
                'transform_timeout': 0.2,


  
                'always_send_full_map': True,
            }]
        )
    ])
