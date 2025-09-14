from setuptools import setup

package_name = 'usb_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='USB camera ROS2 node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'usb_camera_node = usb_camera.usb_camera_node:main',
        ],
    },
)
