# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Stock GNSS profile: UM982 lifecycle driver and the NTRIP/GGA loop."""

from pathlib import Path

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
    driver = FindPackageShare('um982_driver')
    params = LaunchConfiguration('params_file').perform(context)
    if not params:
        params = PathJoinSubstitution(
            [driver, 'config', 'um982_rover.yaml']).perform(context)
    ntrip = LaunchConfiguration('ntrip_params_file').perform(context)
    if not ntrip:
        ntrip = PathJoinSubstitution([
            FindPackageShare('ntrip_client'), 'config', 'ntrip.yaml.example',
        ]).perform(context)
    if not Path(ntrip).is_file():
        raise FileNotFoundError(f'NTRIP parameter file not found: {ntrip}')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([driver, 'launch', 'gnss_rtk.launch.py'])),
        launch_arguments={
            'um982_params_file': params,
            'port': LaunchConfiguration('port'),
            'ntrip_params_file': ntrip,
            'rtcm_topic': LaunchConfiguration('rtcm_topic'),
            'auto_activate': LaunchConfiguration('auto_activate'),
        }.items(),
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('ntrip_params_file', default_value=''),
        DeclareLaunchArgument('rtcm_topic', default_value='/rtcm'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
