# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Non-executing test profile; test_vendor_driver is not a real driver."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    role = LaunchConfiguration('sensor_role').perform(context)
    params = LaunchConfiguration('params_file').perform(context)
    if not params:
        params = str(Path(__file__).with_name('alternate_sensor.yaml'))
    parameters = [params, {'frame_id': role + '_link'}]
    port = LaunchConfiguration('port').perform(context)
    if port:
        parameters.append({'device_uri': port})
    remappings = {
        'gnss': [
            ('fix', '/um982_driver/fix'),
            ('heading', '/um982_driver/heading'),
            ('nmea', '/um982_driver/nmea_sentence'),
            ('rtcm_in', '/rtcm'),
        ],
        'imu': [('imu', '/imu_driver/data')],
        'lidar': [('laser', '/scan_raw')],
    }
    return [Node(
        package='test_vendor_driver',
        executable='test_sensor',
        name='replacement_' + role,
        parameters=parameters,
        remappings=remappings[role],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
