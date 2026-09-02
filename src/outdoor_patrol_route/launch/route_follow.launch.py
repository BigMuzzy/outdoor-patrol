# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
r"""Follow a recorded route with pure pursuit + shoulder retreat.

Runs `route_follower` alongside an already-running GNSS stack; it does not
start one. The follower publishes to /cmd_vel_raw, so the M3 forward brake
stays in the command path underneath it -- start the stack with safety on.

    ros2 launch outdoor_patrol_route route_follow.launch.py \\
        route_path:=/tmp/route.yaml use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('outdoor_patrol_route')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'route.yaml']),
            description='Parameter file for route_follower.'),
        DeclareLaunchArgument(
            'route_path',
            description='Route file to follow (required).'),
        DeclareLaunchArgument(
            'nominal_speed_ms', default_value='0.8'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
    ]

    follower = Node(
        package='outdoor_patrol_route',
        executable='route_follower',
        name='route_follower',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'route_path': LaunchConfiguration('route_path'),
                'nominal_speed_ms': LaunchConfiguration('nominal_speed_ms'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription(args + [follower])
