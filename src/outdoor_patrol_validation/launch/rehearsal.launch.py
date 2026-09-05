# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Rehearse the field validation indoors, against a synthetic stack.

    ros2 launch outdoor_patrol_validation rehearsal.launch.py
    ros2 launch outdoor_patrol_validation rehearsal.launch.py scenario:=obstacle

Brings up the fake robot, the gate node and the one-screen RViz layout
together, so the whole dashboard can be driven end to end on a desk. Use it to
learn the panel and to confirm the gates are configured for your site before
the trip -- see ``field_rehearsal_node`` for what each scenario is meant to
prove.

``soak_hold_s`` is dropped to 30 s here. Rehearsing Phase 1 at its real ten
minutes teaches you nothing you cannot learn in thirty seconds, but do not
carry this override into the field: that hold IS the gate.

Do not run this while the real robot is up. It publishes on the real topic
names and would fight the drivers for them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('outdoor_patrol_validation')
    bringup = FindPackageShare('outdoor_patrol_bringup')

    arguments = [
        DeclareLaunchArgument(
            'scenario', default_value='nominal',
            description='nominal | heading_flip | bad_rtk | obstacle | '
                        'wrong_side | lever_arm_flipped | gnss_fault | '
                        'driveway'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open the one-screen RViz layout with the panel.'),
        DeclareLaunchArgument(
            'start_delay_s', default_value='12.0',
            description='Stationary time before the run starts, so Phase 0 '
                        'and Phase 4 have something to settle on.'),
        DeclareLaunchArgument(
            'soak_hold_s', default_value='30.0',
            description='Phase 1 hold. 600 s in the field; short here so the '
                        'rehearsal does not take ten minutes.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [pkg, 'config', 'field_dashboard.yaml']),
            description='Dashboard profile. Use field_dashboard_driveway.yaml '
                        'with scenario:=driveway, or the gates will be the '
                        'alley ones and a good circuit will fail.'),
    ]

    # The real URDF, so the 3D view shows the actual robot and the real
    # lidar_link / gnss_link mounts rather than the rehearsal's copies of
    # them. With this running the node must not publish them too.
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [bringup, 'launch', 'description.launch.py'])))

    rehearsal = Node(
        package='outdoor_patrol_validation',
        executable='field_rehearsal',
        name='field_rehearsal',
        output='screen',
        parameters=[{
            'scenario': LaunchConfiguration('scenario'),
            'start_delay_s': LaunchConfiguration('start_delay_s'),
            'publish_static_tf': False,
        }],
    )

    dashboard = Node(
        package='outdoor_patrol_validation',
        executable='field_dashboard',
        name='field_dashboard',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'site': 'rehearsal',
             'thresholds.soak_hold_s': LaunchConfiguration('soak_hold_s')},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=[
            '-d', PathJoinSubstitution(
                [pkg, 'config', 'field_validation.rviz'])],
    )

    return LaunchDescription(
        arguments + [description, rehearsal, dashboard, rviz])
