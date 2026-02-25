"""Main launch file for the Sabertooth motor controller node.

Usage:
    # Start with default params (auto-detect hardware):
    ros2 launch sabertooth_motor_controller motor_controller.launch.py

    # Start in simulation mode (no hardware required):
    ros2 launch sabertooth_motor_controller motor_controller.launch.py simulation_mode:=true

    # Use custom config file:
    ros2 launch sabertooth_motor_controller motor_controller.launch.py \
        config_file:=/path/to/custom_params.yaml

    # Use 2x32 config:
    ros2 launch sabertooth_motor_controller motor_controller.launch.py \
        config_file:=$(ros2 pkg prefix sabertooth_motor_controller)/share/sabertooth_motor_controller/config/sabertooth_2x32_params.yaml

    # Debug logging:
    ros2 launch sabertooth_motor_controller motor_controller.launch.py log_level:=debug
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('sabertooth_motor_controller')

    # === Launch Arguments ===

    sim_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation mode without hardware (auto-detected if false)'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'default_params.yaml'),
        description='Path to parameter YAML file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level: debug, info, warn, error'
    )

    # === Motor Controller Node ===

    motor_controller_node = Node(
        package='sabertooth_motor_controller',
        executable='motor_controller_node',
        name='sabertooth_motor_controller',
        parameters=[
            LaunchConfiguration('config_file'),
            {'hardware.simulation_mode': LaunchConfiguration('simulation_mode')},
        ],
        output='screen',
        emulate_tty=True,
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level'),
        ],
    )

    return LaunchDescription([
        sim_mode_arg,
        config_file_arg,
        log_level_arg,
        motor_controller_node,
    ])
