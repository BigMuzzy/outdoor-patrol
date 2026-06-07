# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the Inertial Labs binary-protocol IMU driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    """Build the node, overriding baudrate only when the baud arg is set."""
    params = [LaunchConfiguration('params_file')]
    baud = LaunchConfiguration('baud').perform(context)
    if baud:
        params.append({'baudrate': int(baud)})

    driver = LifecycleNode(
        package='imu_driver',
        executable='imu_driver_node',
        name='imu_driver',
        namespace='',
        output='screen',
        parameters=params,
    )
    return [driver]


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('imu_driver')
    default_params = PathJoinSubstitution([pkg, 'config', 'imu_driver.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the IMU driver parameter YAML.',
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='',
            description='Override the serial baudrate from params_file '
                        '(e.g. baud:=2000000). Empty keeps the YAML value.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
