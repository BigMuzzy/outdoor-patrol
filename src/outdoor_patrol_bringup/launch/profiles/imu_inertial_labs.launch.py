# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Stock IMU profile for the binary protocol implemented by imu_driver."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    driver = FindPackageShare('imu_driver')
    params = LaunchConfiguration('params_file').perform(context)
    if not params:
        params = PathJoinSubstitution(
            [driver, 'config', 'imu_driver.yaml']).perform(context)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([driver, 'launch', 'imu_driver.launch.py'])),
        launch_arguments={
            'params_file': params,
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'auto_activate': LaunchConfiguration('auto_activate'),
            'use_static_tf': 'false',
            'use_rviz': 'false',
        }.items(),
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('baud', default_value=''),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
