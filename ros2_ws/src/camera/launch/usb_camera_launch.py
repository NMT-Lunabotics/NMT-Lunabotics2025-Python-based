from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import yaml, os

def launch_setup(context):
    """Launches a topic for each camera specified in cameras.yaml, debug/system use. And starts camera stream with feed switching for userinterface use"""
    camera_config = context.launch_configurations['camera_config']
    with open(camera_config, 'r') as f: config = yaml.safe_load(f)

    nodes = []
    shutdown_handlers = []
    for cam in config['cameras']:
        # v4l2 camera node publishes raw video data
        camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace=cam['name'],
            name='v4l2_camera_node',
            output='screen',
            parameters=[{
                'video_device': cam['video_device'],
                'image_size': [cam['width'],cam['height']],
                'framerate': int(cam['frame_rate']),
                'image_encoding': 'rgb8'
            }]
        )

        # Republish node ensures compressed video feed exists
        republish_node = Node(
            package='image_transport',
            executable='republish',
            name=f'{cam["name"]}_compressed',
            arguments=[
                'raw', 'compressed',
                '--ros-args',
                '--remap', f'in:=/{cam["name"]}/image_raw',
                '--remap', f'out/compressed:=/{cam["name"]}/compressed'
            ],
            parameters=[{'jpeg_quality': 0}]
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
    mux_fps = float(config['cameras'][0]['frame_rate'])
    nodes.append(
        Node(
            package='camera',
            executable='camera_mux_node.py',
            name='camera_mux_node',
            parameters=[{
                'output_fps': mux_fps
            }]
        )
    )
    return nodes + shutdown_handlers

def generate_launch_description():
    default_camera_yaml = os.path.join(get_package_share_directory('camera'),'config','pc_cameras.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_config',default_value=default_camera_yaml, description='Path to cameras.yaml configuration file'),
        OpaqueFunction(function=launch_setup)
    ])