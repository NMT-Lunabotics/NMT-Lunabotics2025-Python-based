from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """Starts controller_input_node for converting button inputs into activation topics"""
    device_id_arg = DeclareLaunchArgument('device_id',default_value='0',description='Joystick device ID')

    device_id = LaunchConfiguration('device_id')
    return LaunchDescription([
        device_id_arg,
        # Start the joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # Adjust the joystick device if necessary
            parameters=[{
                'device_id': device_id,
                'deadzone': 0.2,
                'autorepeat_rate': 10.0,  # Set the publish rate in Hz
                'coalesce_interval_ms': 100,
            }],
            additional_env={'ROS_DOMAIN_ID': '10'}
        ),

        # Start the controller_input node
        Node(
            package='controller_input',
            executable='controller_input_node.py',
            name='controller_input_node',
            output='screen',
            parameters=[{
                'require_enable_button': True,
                'inverted_reverse': False
            }],
            additional_env={'ROS_DOMAIN_ID': '10'}
        )
    ])