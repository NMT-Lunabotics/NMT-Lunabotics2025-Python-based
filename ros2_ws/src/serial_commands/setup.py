from setuptools import setup

package_name = 'serial'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Benjamin Peterson',
    maintainer_email='benjamin.peterson@student.nmt.edu',
    description='ROS2 Humble serial command node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_writer_node = serial_command.serial_writer_node:main',
            'serial_listener_node = serial_command.serial_listener_node:main',
        ],
    },
)
