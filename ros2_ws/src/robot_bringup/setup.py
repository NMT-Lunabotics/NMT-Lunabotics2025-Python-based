from setuptools import setup
from glob import glob
import os

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='cameron@todo.todo',
    description='Robot bringup package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'odom_bridge = robot_bringup.odom_bridge:main',
            'robot_mask_filter = robot_bringup.robot_mask_filter:main',
        ],
    },
)
