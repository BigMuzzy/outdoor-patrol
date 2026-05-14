# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Publish robot_state_publisher with the M0 URDF.

Reusable leaf for all later milestones: any launch file that needs the
robot description includes this one.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('outdoor_patrol_bringup')

    default_xacro = PathJoinSubstitution(
        [pkg, 'urdf', 'outdoor_patrol.urdf.xacro'])

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_xacro,
        description='Absolute path to the robot URDF xacro file.')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', LaunchConfiguration('urdf_file')]),
            value_type=str),
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        urdf_file_arg,
        use_sim_time_arg,
        robot_state_publisher,
    ])
