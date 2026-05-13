from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav_stream = LaunchConfiguration('nav_stream')
    # Get file paths of diffrent packages
    camera_directory=get_package_share_directory('camera')
    system_start_directory=get_package_share_directory('system_start')

    # Get file paths of needed files from directorys
    camera_launch_file=os.path.join(camera_directory,'launch','usb_camera_launch.py')
    camera_config_file=os.path.join(system_start_directory,'config','cameras.yaml')

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

    waypoint_node = Node(
        package='point_navigation',
        executable='waypoint.py',
        name='waypoint'
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

    nav2_params_file = os.path.join(
        get_package_share_directory('point_navigation'),
        'config',
        'nav2_params.yaml'
    )

    nav2_launch = ExecuteProcess(
        cmd=[
            "ros2", "launch", "nav2_bringup", "navigation_launch.py",
            "use_sim_time:=false",
            f"params_file:={nav2_params_file}"
        ],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',
            os.path.join(
                get_package_share_directory('point_navigation'),
                'config',
                'nav.rviz'
            )
        ],
        output='screen'
    )

    serial_talker_node = Node(
        package='serial_commands',
        executable='serial_talker_node.py',
        name='serial_talker_node'
    )

    automated_operations = Node(
        package='automated_operations',
        executable='automated_operations_node.py',
        name='automated_operations_node'
    )

    camera_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={
            'camera_config': camera_config_file, 
            'nav_stream': nav_stream
        }.items()
    )

    return LaunchDescription([
        realsense_launch,
        base_frame,
        rtabmap_launch,
        #rviz_node,
        waypoint_node,
        nav2_launch,
        camera_launch,
        automated_operations,
        serial_talker_node
    ])