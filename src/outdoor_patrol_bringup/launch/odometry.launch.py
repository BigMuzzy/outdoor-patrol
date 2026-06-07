# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""M1 bringup: chassis odometry + EKF TF tree.

Builds on the M0 teleop stack (micro-ROS agent + robot_state_publisher)
and adds the single-input robot_localization EKF from outdoor_patrol_loc,
which consumes the chassis `/odom` and broadcasts `odom -> base_link` plus
`/odometry/filtered`.

Drive with the keyboard in a second terminal (real TTY required):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Visualize with the M1 preset:

    ros2 launch outdoor_patrol_bringup rviz.launch.py \\
        rviz_config:=$(ros2 pkg prefix outdoor_patrol_bringup)/share/outdoor_patrol_bringup/config/odometry.rviz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = FindPackageShare('outdoor_patrol_bringup')
    loc_pkg = FindPackageShare('outdoor_patrol_loc')

    serial_dev_arg = DeclareLaunchArgument(
        'serial_dev',
        default_value='/dev/ttyACM0',
        description='Serial device exposed by the ESP32-S3 micro-ROS agent.')

    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud',
        default_value='115200',
        description='Baud rate passed to micro_ros_agent (CDC ignores it).')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.')

    # M0 teleop stack: micro-ROS agent (publishes /odom, consumes /cmd_vel)
    # and robot_state_publisher.
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_pkg, 'launch', 'teleop.launch.py'])),
        launch_arguments={
            'serial_dev': LaunchConfiguration('serial_dev'),
            'serial_baud': LaunchConfiguration('serial_baud'),
        }.items(),
    )

    # M1 localization: single-input EKF -> odom -> base_link TF.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([loc_pkg, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        serial_dev_arg,
        serial_baud_arg,
        use_sim_time_arg,
        teleop_launch,
        localization_launch,
    ])
