from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'auv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-mad',
    maintainer_email='2021ci62f@sigce.edu.in',
    description='ros2_control controllers and cmd_vel mapping for the Pralay-2 fins and propeller (hardware and Gazebo)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "fins_control = auv_control.fins_control:main",
            "cmd_vel_to_actuators = auv_control.cmd_vel_to_actuators:main",
            "depth_hold = auv_control.depth_hold:main",
            "simple_depth_fin = auv_control.simple_depth_fin:main",
            "simple_roll_fin = auv_control.simple_roll_fin:main",
        ],
    },
)
