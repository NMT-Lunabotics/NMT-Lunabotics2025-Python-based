from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   # static_lidar_to_body = Node(
   #     package='tf2_ros',
   #     executable='static_transform_publisher',
   #     name='base_to_lidar',
   #     arguments=[
   #         '0', '0', '0',      # x y z
   #         '0', '0', '0',     # roll pitch yaw
   #         'body',
   #         'odom',
   #     ],
   # )
   # 
    static_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=[
            '0', '0', '0',      # x y z
            '0', '0', '0',     # roll pitch yaw
            'map',
            'odom',
        ],
    )
    return LaunchDescription([
    #   static_lidar_to_body,
        static_odom_to_base_link
    ])



