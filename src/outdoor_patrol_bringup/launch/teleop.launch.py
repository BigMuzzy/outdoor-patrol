# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""M0 teleop bringup.

Starts the micro-ROS agent (USB-CDC to the ESP32-S3 chassis controller)
and robot_state_publisher. Keyboard teleop is launched manually in a
second terminal because teleop_twist_keyboard requires a real TTY for
getch():

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('outdoor_patrol_bringup')

    # USB-CDC default per esp32-s3-uros-controller ADR-0002.
    serial_dev_arg = DeclareLaunchArgument(
        'serial_dev',
        default_value='/dev/ttyACM0',
        description='Serial device exposed by the ESP32-S3 micro-ROS agent.')

    # Baud is ignored over USB-CDC but the agent CLI still requires it.
    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud',
        default_value='115200',
        description='Baud rate passed to micro_ros_agent (CDC ignores it).')

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'description.launch.py'])),
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=[
            'serial',
            '--dev', LaunchConfiguration('serial_dev'),
            '-b', LaunchConfiguration('serial_baud'),
        ],
    )

    return LaunchDescription([
        serial_dev_arg,
        serial_baud_arg,
        description_launch,
        micro_ros_agent,
    ])
