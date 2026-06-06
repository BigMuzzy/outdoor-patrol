# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""M1 localization: single-input robot_localization EKF.

Brings up `ekf_node` consuming the chassis wheel odometry (`/odom`) and
broadcasting `odom -> base_link` + `/odometry/filtered`. This is the sole
owner of the `odom -> base_link` transform; the ESP32-S3 firmware publishes
the odom message only (no TF). M2 extends the same node with an IMU.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('outdoor_patrol_loc')
    default_params = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the robot_localization EKF parameter YAML.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        ekf_node,
    ])
