# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""One-screen field validation dashboard: gate node + RViz panel.

    ros2 launch outdoor_patrol_validation field_dashboard.launch.py

Run it from the **dev box**, not the robot: the panel needs a display, and the
node only reads topics. If you want the gates accumulating even when nobody
has RViz open -- which is the safe default for a long soak -- run it on the
robot with ``rviz:=false`` as well, and take the report off the robot with the
rest of the run data.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('outdoor_patrol_validation')

    arguments = [
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open the one-screen RViz layout with the panel.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [pkg, 'config', 'field_validation.rviz']),
            description='RViz layout to load.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [pkg, 'config', 'field_dashboard.yaml']),
            description='Thresholds and topic names for the gate node.'),
        DeclareLaunchArgument(
            'site', default_value='alley',
            description='Recorded in the report header.'),
        DeclareLaunchArgument(
            'report_dir', default_value='runs/field',
            description='Where the markdown report is written.'),
    ]

    dashboard = Node(
        package='outdoor_patrol_validation',
        executable='field_dashboard',
        name='field_dashboard',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'site': LaunchConfiguration('site'),
             'report_dir': LaunchConfiguration('report_dir')},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription(arguments + [dashboard, rviz])
