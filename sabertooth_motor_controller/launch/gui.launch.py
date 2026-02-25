"""Launch file for the motor controller web GUI.

Starts both the motor controller node and the web GUI node.
Open http://localhost:5000 in a browser after launching.

Usage:
  ros2 launch sabertooth_motor_controller gui.launch.py simulation_mode:=true
  ros2 launch sabertooth_motor_controller gui.launch.py gui_port:=8080
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('sabertooth_motor_controller')

    sim_mode_arg = DeclareLaunchArgument(
        'simulation_mode', default_value='false',
        description='Run motor controller in simulation mode')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'default_params.yaml'),
        description='Path to parameter YAML file')

    port_arg = DeclareLaunchArgument(
        'gui_port', default_value='5000',
        description='Port for the web GUI server')

    # Include the motor controller launch
    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'motor_controller.launch.py')),
        launch_arguments={
            'simulation_mode': LaunchConfiguration('simulation_mode'),
            'config_file': LaunchConfiguration('config_file'),
        }.items())

    # GUI node
    gui_node = Node(
        package='sabertooth_motor_controller',
        executable='gui_node',
        name='motor_controller_gui',
        parameters=[{
            'port': LaunchConfiguration('gui_port'),
        }],
        output='screen',
        emulate_tty=True)

    return LaunchDescription([
        sim_mode_arg,
        config_file_arg,
        port_arg,
        motor_controller_launch,
        gui_node,
    ])
