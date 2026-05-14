# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Launch rviz2 with the M0 teleop preset."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('outdoor_patrol_bringup')

    default_rviz = PathJoinSubstitution([pkg, 'config', 'teleop.rviz'])

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Absolute path to the RViz config file.')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz2,
    ])
