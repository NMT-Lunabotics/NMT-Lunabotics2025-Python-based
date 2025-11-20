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

    rtabmap_launch_file = os.path.join(
        get_package_share_directory(
        'rtabmap_launch'), 
        'launch', 
        'rtabmap.launch.py'
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
        # Solver params
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
        'ceres_loss_function': 'None',

        # ROS Parameters
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_link',
        'scan_topic': '/scan',
        'use_map_saver': True,
        'mode': 'localization',

        # Performance
        'debug_logging': False,
        'throttle_scans': 1,
        'transform_publish_period': 0.02,
        'map_update_interval': 0.1,
        'resolution': 0.05,
        'min_laser_range': 0.0,
        'max_laser_range': 20.0,
        'minimum_time_interval': 0.5,
        'transform_timeout': 0.2,
        'tf_buffer_duration': 60.0,
        'stack_size_to_use': 40000000,
        'enable_interactive_mode': True,

        # General Parameters
        'use_scan_matching': True,
        'use_scan_barycenter': True,
        'minimum_travel_distance': 0.5,
        'minimum_travel_heading': 0.5,
        'scan_buffer_size': 100,
        'scan_buffer_maximum_scan_distance': 10.0,
        'link_match_minimum_response_fine': 0.1,
        'link_scan_maximum_distance': 1.5,
        'loop_search_maximum_distance': 3.0,
        'do_loop_closing': True,
        'loop_match_minimum_chain_size': 10,
        'loop_match_maximum_variance_coarse': 3.0,
        'loop_match_minimum_response_coarse': 0.35,
        'loop_match_minimum_response_fine': 0.45,

        # Correlation Parameters
        'correlation_search_space_dimension': 0.5,
        'correlation_search_space_resolution': 0.01,
        'correlation_search_space_smear_deviation': 0.1,

        # Loop Closure Parameters
        'loop_search_space_dimension': 8.0,
        'loop_search_space_resolution': 0.05,
        'loop_search_space_smear_deviation': 0.03,

        # Scan Matcher Parameters
        'distance_variance_penalty': 0.5,
        'angle_variance_penalty': 1.0,
        'fine_search_angle_offset': 0.00349,
        'coarse_search_angle_offset': 0.349,
        'coarse_angle_resolution': 0.0349,
        'minimum_angle_penalty': 0.9,
        'minimum_distance_penalty': 0.5,
        'use_response_expansion': True,
    }]
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file),
        launch_arguments={
            'scan_topic': '/scan',
            
            # RTAB Map Functionality
            'localization': 'true',
            'approx_sync': 'true',
            # 'mapping': 'true',
            'visual_odometry': 'true',

            # RTAB Map Frames and TFs
            'publish_tf': 'true',
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # Occupancy Grid
            # 'Grid/FromDepth': 'true',
            # 'Grid/3D': 'false',
            # 'Grid/CellSize': '0.05',
            # # 'Grid/DepthDecimation': '5' # Change as needed
            # 'Grid/RangeMin': '0.3',
            # 'Grid/RangeMax': '5.0',
            'grid_map_publisher_rate': '1.0',
            # 'grid_frame_id': 'map',

            # Publishing/Subscribing
            'queue_size': '10',
            'wait_imu_to_init': 'false',
            'subscribe_depth': 'false',
            'subscribe_rgb': 'false',
            'subscribe_stereo': 'false',
            'subscribe_scan': 'true',
            # 'Mem/IncrementalMemory': 'true',
            'rtabmap_args':
                '--ros-args --params-file '
                '/home/luna/ros2_ws/src/slam_config/config/rtabmap_config.yaml'
            
    }.items()
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rplidar_launch,
        laser_tf_node,
        odom_to_base_node,
        slam_toolbox_node,
        rtabmap_launch,
    ])
