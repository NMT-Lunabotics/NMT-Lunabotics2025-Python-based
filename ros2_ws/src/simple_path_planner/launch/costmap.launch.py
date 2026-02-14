from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('simple_path_planner')
    costmap_yaml = os.path.join(pkg, 'config', 'costmap.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'local_costmap.rviz')

    return LaunchDescription([
        Node(
            package='fake_lidar',
            executable='fake_lidar_node.py',
            name='fake_lidar',
            output='screen'
        ),
        Node(
            package='simple_path_planner',
            executable='fake_odom_static.py',
            name='fake_odom_static',
            output='screen'
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            parameters=[costmap_yaml],
            output='screen'
        ),
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/costmap', 'configure'],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/costmap', 'activate'],
                    output='screen'
                )
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen'
        )
    ])
