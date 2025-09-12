from setuptools import setup

package_name = 'simple_chat'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A simple ROS 2 chat package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chat_node = simple_chat.chat_node:main',
        ],
    },
)
