from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sabertooth_motor_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'sabertooth_motor_controller': ['web/*'],
    },
    data_files=[
        # Package index marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rico',
    maintainer_email='rico@todo.todo',
    description='ROS2 Humble driver for Sabertooth 2x16/2x32 motor controller via PCA9685 PWM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = sabertooth_motor_controller.motor_controller_node:main',
            'gui_node = sabertooth_motor_controller.gui_node:main',
        ],
    },
)
