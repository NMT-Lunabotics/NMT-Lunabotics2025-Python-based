from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import yaml

def launch_setup(context):
    """Launches a topic for each camera specified in cameras.yaml, debug/system use. And starts camera stream with feed switching for userinterface use"""
    camera_config = context.launch_configurations['camera_config']
    with open(camera_config, 'r') as f: config = yaml.safe_load(f)

    nodes = []
    shutdown_handlers = []
    for cam in config['cameras']:
        # v4l2 camera node publishes raw feed
        camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace=cam['name'],
            name='v4l2_camera_node',
            output='screen',
            parameters=[{
                'video_device': cam['video_device'],
                'image_width': cam['width'],
                'image_height': cam['height'],
                'frame_rate': cam['frame_rate'],
            }],
            additional_env={'ROS_DOMAIN_ID': '11'}
        )

        # Republish node creates compressed version of raw feed
        republish_node = Node(
            package='image_transport',
            executable='republish',
            namespace=cam['name'],
            name='republish_compressed',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', 'image_raw'),                   # from camera node
                ('out', 'image_raw/compressed')        # compressed topic
            ],
            additional_env={'ROS_DOMAIN_ID': '11'}
        )
        
        # Allow shutdown of nodes when terminal closes
        shutdown_handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=camera_node,
                    on_exit=[EmitEvent(event=Shutdown())]
                )
            )
        )

        nodes.extend([camera_node, republish_node])

    # launch camera muiliplexer
    nodes.append(
        Node(
            package='camera',
            executable='camera_mux_node.py',
            name='camera_mux_node',
            additional_env={'ROS_DOMAIN_ID': '11'}
        )
    )
    return nodes + shutdown_handlers

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('camera_config'),OpaqueFunction(function=launch_setup)])