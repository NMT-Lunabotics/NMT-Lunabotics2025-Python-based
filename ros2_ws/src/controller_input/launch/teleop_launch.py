from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Starts controller_input_node for converting button inputs into activation topics"""
    # Launch file paths
    default_controller_schematic = os.path.join(get_package_share_directory('controller_input'),'config','controller_schematic.yaml')
    default_controller_config = os.path.join(get_package_share_directory('controller_input'),'config','controller_config.yaml')
    default_controller_automations = os.path.join(get_package_share_directory('controller_input'),'config','controller_automate.yaml')

    # Load launch arguments
    controller_config = LaunchConfiguration('controller_config')

    # Launch nodes
    return LaunchDescription([
        # Load launch files
        DeclareLaunchArgument('controller_config',default_value=default_controller_config, description='Path to controller_config.yaml configuration file'),
        DeclareLaunchArgument('device_id',default_value='0',description='Joystick device ID'),

        # Start the joystick node
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            output='screen',
            parameters=[ controller_config ],
        ),

        # Start the controller_input node
        Node(
            package='controller_input',
            executable='controller_input_node.py',
            name='controller_input_node',
            output='screen',
            parameters=[
                {
                    'controllers': default_controller_schematic,
                    'config': default_controller_config,
                    'automations': default_controller_automations,
                    'require_enable_button': True,
                    'inverted_reverse': False
                }
            ],
        )
    ])