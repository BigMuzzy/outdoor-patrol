# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Record a teach pass into a route file.

Runs `route_recorder` alongside an already-running GNSS stack -- it does not
start one. Bring the stack up first (gnss_localization.launch.py on the robot,
sim.launch.py in simulation), drive the route, then::

    ros2 service call /route_recorder/save std_srvs/srv/Trigger

The default source takes base_link straight off /odometry/global. Pass
`source:=fix_lever_arm` for the independent cross-check, or
`source:=raw_antenna` for the uncorrected control file used by the
differential test.
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
            description='Parameter file for route_recorder.'),
        DeclareLaunchArgument(
            'output_path', default_value='route.yaml',
            description='Where ~/save writes the route file.'),
        DeclareLaunchArgument(
            'source', default_value='odometry_global',
            description='odometry_global | fix_lever_arm | raw_antenna.'),
        DeclareLaunchArgument(
            'loop', default_value='true',
            description='Request loop closure. The recorder verifies it and '
                        'downgrades to false if the pass did not close.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
    ]

    recorder = Node(
        package='outdoor_patrol_route',
        executable='route_recorder',
        name='route_recorder',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'output_path': LaunchConfiguration('output_path'),
                'source': LaunchConfiguration('source'),
                'loop': LaunchConfiguration('loop'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription(args + [recorder])
