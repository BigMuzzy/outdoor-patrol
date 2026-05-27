# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the WitMotion UM982 RTK GNSS/INS driver."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('witmotion_um982')
    default_params = PathJoinSubstitution(
        [pkg, 'config', 'um982.yaml']
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to UM982 driver parameter YAML.',
    )

    driver = Node(
        package='witmotion_um982',
        executable='um982_driver',
        name='um982_driver',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, driver])
