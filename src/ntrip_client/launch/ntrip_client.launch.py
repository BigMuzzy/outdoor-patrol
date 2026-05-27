# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the NTRIP caster client."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('ntrip_client')
    default_params = PathJoinSubstitution([pkg, 'config', 'ntrip.yaml.example'])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to NTRIP caster credentials YAML.',
    )

    return LaunchDescription([
        params_arg,
        Node(
            package='ntrip_client',
            executable='ntrip_client_node',
            name='ntrip_client',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
