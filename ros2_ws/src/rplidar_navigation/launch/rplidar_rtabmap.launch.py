from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'rplidar_port', 
            default_value='/dev/ttyUSB0',
            description='RPLIDAR serial port'),
        
        # RPLIDAR A3 driver - ULTRA REDUCED RATE
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': LaunchConfiguration('rplidar_port'),
                'serial_baudrate': 256000,
                'frame_id': 'laser_link',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Express',  # Express mode is slower than Standard
            }],
            output='screen'
        ),
        
        # Static transform from base to laser - DELAYED START
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_to_laser',
                    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link']
                ),
            ]
        ),
        
        # SLAM Toolbox - ULTRA AGGRESSIVE THROTTLING
        TimerAction(
            period=5.0,  # Wait for RPLIDAR to stabilize
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',  # MUST USE SYNC VERSION
                    name='slam_toolbox',
                    parameters=[{
                        'odom_frame': 'odom',
                        'map_frame': 'map', 
                        'base_frame': 'base_link',
                        'scan_topic': '/scan',
                        'use_sim_time': False,
                        
                        # CRITICAL: Ultra aggressive throttling
                        'throttle_scans': 10,  # Process only every 10th scan
                        'transform_timeout': 0.5,
                        'tf_buffer_duration': 20.0,  # Much larger buffer
                        
                        # Map settings
                        'resolution': 0.1,  # Lower resolution for speed
                        'max_laser_range': 20.0,  # Reduced range
                        
                        # Performance - very conservative
                        'minimum_travel_distance': 0.5,  # Only update after 50cm movement
                        'minimum_travel_heading': 0.5,   # Only update after significant rotation
                        'map_update_interval': 5.0,      # Only update map every 5 seconds
                    }],
                    output='screen'
                ),
            ]
        ),
        
        # RViz2 - DELAYED START
        TimerAction(
            period=10.0,  # Wait for everything else to stabilize
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', '/opt/ros/humble/share/slam_toolbox/config/slam_toolbox_default.rviz'],
                    parameters=[{
                        'use_sim_time': False
                    }],
                    output='screen'
                ),
            ]
        ),
    ])