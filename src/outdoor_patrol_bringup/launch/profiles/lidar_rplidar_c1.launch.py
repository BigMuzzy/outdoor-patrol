# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Stock serial RPLIDAR C1 profile; filtering and braking belong to bringup."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    params = LaunchConfiguration('params_file').perform(context)
    if not params:
        params = PathJoinSubstitution([
            FindPackageShare('outdoor_patrol_bringup'), 'config', 'lidar.yaml',
        ]).perform(context)
    if not Path(params).is_file():
        raise FileNotFoundError(f'LiDAR parameter file not found: {params}')
    parameters: list[str | dict[str, str]] = [params]
    port = LaunchConfiguration('port').perform(context)
    if port:
        parameters.append({'serial_port': port})
    return [Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=parameters,
        remappings=[('scan', '/scan_raw')],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
