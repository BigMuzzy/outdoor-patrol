# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the 2D-LiDAR forward safety brake (M3, ADR-013).

Sits between the command source and the chassis: remaps ``cmd_vel_in`` from
``/cmd_vel_raw`` (teleop / Nav2) and publishes the gated ``cmd_vel_out`` to
``/cmd_vel`` (the chassis input).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the scan_safety launch description."""
    pkg = FindPackageShare('outdoor_patrol_safety')
    default_params = PathJoinSubstitution([pkg, 'config', 'scan_safety.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='scan_safety parameter YAML.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Input LaserScan topic.'),
        DeclareLaunchArgument(
            'cmd_vel_in', default_value='/cmd_vel_raw',
            description='Raw (ungated) command input.'),
        DeclareLaunchArgument(
            'cmd_vel_out', default_value='/cmd_vel',
            description='Gated command output to the chassis.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Drive the re-gate timer and the scan/command '
                        'timeouts off /clock. Must be true under Gazebo.'),
        Node(
            package='outdoor_patrol_safety',
            executable='scan_safety',
            name='scan_safety',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic')),
                ('cmd_vel_in', LaunchConfiguration('cmd_vel_in')),
                ('cmd_vel_out', LaunchConfiguration('cmd_vel_out')),
            ],
        ),
    ])
