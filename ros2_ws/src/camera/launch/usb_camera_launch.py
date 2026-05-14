from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.logging import get_logger
import yaml, os

REALSENSE_SKIP_ID="_306322300659"

def launch_setup(context):
    """Launches a topic for each camera specified in cameras.yaml, debug/system use. And starts camera stream with feed switching for userinterface use"""
    logger = get_logger('camera_launch')
    camera_config = context.launch_configurations['camera_config']
    nav_stream = LaunchConfiguration('nav_stream').perform(context).lower() == "true"
    with open(camera_config, 'r') as f: config = yaml.safe_load(f)

    nodes = []
    shutdown_handlers = []
    camera_id=-1
    for cam in config['cameras']:
        cam_id = int(cam["name"].replace("camera", ""))
        print("======================================================")
        print(f"{cam_id}")

        if nav_stream and cam.get("serial_no") == REALSENSE_SKIP_ID:
            camera_id = cam_id
            continue
        # v4l2 camera node publishes raw video data
        if not os.path.exists(cam['video_device']):
             if not os.path.exists(cam['video_device']):
                logger.warning(f"\033[33mCamera device {cam['video_device']} "f"does not exist. Skipping {cam['name']}\033[0m")
                continue
        camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace=cam['name'],
            name='v4l2_camera_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'error'],
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
        #shutdown_handlers.append(
        #    RegisterEventHandler(
        #        OnProcessExit(
        #            target_action=camera_node,
        #            on_exit=[EmitEvent(event=Shutdown())]
        #        )
        #    )
        #)

        nodes.extend([camera_node, republish_node])

    # launch camera muiliplexer
    mux_fps = float(config['cameras'][0]['frame_rate'])
    nodes.append(
        Node(
            package='camera',
            executable='camera_mux_node.py',
            name='camera_mux_node',
            parameters=[{
                'output_fps': mux_fps,
                'nav_stream': str(camera_id)
            }]
        )
    )
    return nodes + shutdown_handlers

def generate_launch_description():
    default_camera_yaml = os.path.join(get_package_share_directory('camera'),'config','pc_cameras.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('nav_stream', default_value='false'),
        DeclareLaunchArgument('camera_config',default_value=default_camera_yaml, description='Path to cameras.yaml configuration file'),
        OpaqueFunction(function=launch_setup)
    ])