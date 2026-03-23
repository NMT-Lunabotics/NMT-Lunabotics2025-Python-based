from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Laucnhes the user side of the ros system, the stuff that will talk over the network (controller, camera viewer)"""
    # Get file paths of diffrent packages
    camera_directory=get_package_share_directory('camera')
    camera_viewer_launch_file=os.path.join(camera_directory,'launch','view_camera_launch.py')

    # Setup node and launch, launchers
    camera_viewer_launch=IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_viewer_launch_file))
    
    controller_launch=Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            arguments=['image:=/camera/stream'],  
            output='screen'
        )
    
    # Launch all specifyied launch files and nodes
    return LaunchDescription([camera_viewer_launch,controller_launch])