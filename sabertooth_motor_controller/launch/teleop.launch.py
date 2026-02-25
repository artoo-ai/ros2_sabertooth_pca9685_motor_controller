"""Launch motor controller with teleop_twist_keyboard for manual testing.

This launch file starts both the motor controller node and the
teleop_twist_keyboard node for interactive manual control.

Usage:
    # On Jetson with hardware:
    ros2 launch sabertooth_motor_controller teleop.launch.py

    # On Mac in simulation mode:
    ros2 launch sabertooth_motor_controller teleop.launch.py simulation_mode:=true

Controls (teleop_twist_keyboard):
    u    i    o        (forward-left, forward, forward-right)
    j    k    l        (turn-left, stop, turn-right)
    m    ,    .        (back-left, backward, back-right)

    q/z: increase/decrease max speed by 10%
    w/x: increase/decrease linear speed by 10%
    e/c: increase/decrease angular speed by 10%

NOTE: teleop_twist_keyboard must be installed:
    sudo apt install ros-humble-teleop-twist-keyboard

For joystick control, use joy_teleop instead:
    ros2 launch teleop_twist_joy teleop-launch.py
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

    # === Launch Arguments ===

    sim_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation mode without hardware'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'default_params.yaml'),
        description='Path to parameter YAML file'
    )

    # === Include Motor Controller Launch ===

    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'motor_controller.launch.py')
        ),
        launch_arguments={
            'simulation_mode': LaunchConfiguration('simulation_mode'),
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
    )

    # === Teleop Keyboard Node ===
    # Publishes geometry_msgs/Twist on /cmd_vel

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        # Remap if needed (default is /cmd_vel which matches our subscription)
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    return LaunchDescription([
        sim_mode_arg,
        config_file_arg,
        motor_controller_launch,
        teleop_node,
    ])
