from setuptools import setup, find_packages

package_name = 'gp_navigation_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='cameron.symonds@student.nmt.edu',
    description='GP traversability nodes (ROS2).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gp_mapping_module = gp_navigation_ros2.gp_mapping_module_node:main', 'gp_explorer = gp_navigation_ros2.gp_explorer_node:main', 'gp_global_mapper = gp_navigation_ros2.gp_global_mapper_node:main',
        ],
    },
)

