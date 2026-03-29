from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Starts controller_input_node for converting button inputs into activation topics"""
    default_controller_schematic = os.path.join(get_package_share_directory('controller_input'),'config','controller_schematic.yaml')
    device_id_arg = DeclareLaunchArgument('device_id',default_value='0',description='Joystick device ID')

    device_id = LaunchConfiguration('device_id')
    controller_schematic = LaunchConfiguration('controller_schematic')
    return LaunchDescription([
        DeclareLaunchArgument('controller_schematic',default_value=default_controller_schematic, description='Path to controller_schematic.yaml configuration file'),
        #SetEnvironmentVariable('ROS_DOMAIN_ID', '10'),
        device_id_arg,
        # Start the joystick node
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            output='screen',
            # Adjust the joystick device if necessary
            parameters=[{
                'device_id': device_id,
                'deadzone': 0.2,
                'autorepeat_rate': 10.0,  # Set the publish rate in Hz
            }],
        ),

        # Start the controller_input node
        Node(
            package='controller_input',
            executable='controller_input_node.py',
            name='controller_input_node',
            output='screen',
            parameters=[
                {
                    'controller_schematic': controller_schematic,
                    'require_enable_button': True,
                    'inverted_reverse': False
                }
            ],
        )
    ])