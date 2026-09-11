# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the UM982 RTK GNSS driver."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    params_file = Path(LaunchConfiguration('params_file').perform(context))
    if not params_file.is_file():
        raise FileNotFoundError(f'UM982 parameter file not found: {params_file}')
    params: list[str | dict[str, str]] = [str(params_file)]
    port = LaunchConfiguration('port').perform(context)
    if port:
        params.append({'port': port})

    return [LifecycleNode(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver',
        namespace='',
        output='screen',
        parameters=params,
    )]


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('um982_driver')
    default_params = PathJoinSubstitution([pkg, 'config', 'um982_rover.yaml'])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to UM982 driver parameter YAML.',
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='',
        description='Override the serial port. Empty keeps the YAML value.',
    )

    return LaunchDescription([
        params_arg, port_arg, OpaqueFunction(function=_launch_setup),
    ])
