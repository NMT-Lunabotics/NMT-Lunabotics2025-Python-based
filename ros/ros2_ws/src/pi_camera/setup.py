from setuptools import setup

package_name = 'pi_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Launch package for Raspberry Pi camera using v4l2_camera',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
