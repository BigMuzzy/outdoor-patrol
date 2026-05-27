# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the serial-radio RTCM relay."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('ntrip_client')
    default_params = PathJoinSubstitution([pkg, 'config', 'local_base_serial.yaml'])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to serial RTCM relay parameter YAML.',
    )

    return LaunchDescription([
        params_arg,
        Node(
            package='ntrip_client',
            executable='serial_rtcm_relay_node',
            name='serial_rtcm_relay',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
