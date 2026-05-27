# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the UM982 RTK GNSS driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('um982_driver')
    default_params = PathJoinSubstitution([pkg, 'config', 'um982_rover.yaml'])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to UM982 driver parameter YAML.',
    )

    driver = LifecycleNode(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, driver])
