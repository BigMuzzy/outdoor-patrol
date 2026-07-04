# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Interim GNSS global-localization bringup (ADR-012) — full stack, one launch.

Composes everything needed to run the interim global localization on the real
robot:

  teleop.launch.py              micro-ROS agent + robot_state_publisher
   [outdoor_patrol_bringup]     -> /odom, /cmd_vel, TF base_link<->gnss_link
  gnss_rtk.launch.py            UM982 driver + NTRIP (RTK), lifecycle
   [um982_driver]               auto-activated -> /um982_driver/fix, /heading
  global_localization.launch.py dual-EKF + heading adapter + confidence_gate +
   [outdoor_patrol_loc]         navsat_transform -> odom->base_link, map->odom
  rviz2 (optional)              fixed frame = map

Drive with the keyboard in a SEPARATE terminal (needs a real TTY):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

NTRIP credentials: pass `ntrip_params_file:=/path/to/ntrip.yaml`; the default
points at the package example (no real caster).

TBD before the field test (integration plan items 2/3): the heading
`yaw_offset` (heading_to_imu) once the antenna-baseline mount angle is
measured — until then the `map` orientation is unaligned. Datum is
auto-on-first-fix (config/navsat.yaml), so start near the dock.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare('outdoor_patrol_bringup')
    loc = FindPackageShare('outdoor_patrol_loc')
    um982 = FindPackageShare('um982_driver')
    ntrip = FindPackageShare('ntrip_client')

    serial_dev = LaunchConfiguration('serial_dev')
    serial_baud = LaunchConfiguration('serial_baud')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ntrip_params_file = LaunchConfiguration('ntrip_params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    args = [
        DeclareLaunchArgument(
            'serial_dev', default_value='/dev/ttyACM0',
            description='Serial device of the ESP32-S3 micro-ROS agent.'),
        DeclareLaunchArgument(
            'serial_baud', default_value='115200',
            description='Baud passed to micro_ros_agent (CDC ignores it).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true.'),
        DeclareLaunchArgument(
            'ntrip_params_file',
            default_value=PathJoinSubstitution(
                [ntrip, 'config', 'ntrip.yaml.example']),
            description='NTRIP caster credentials YAML. Override with your '
                        'real ntrip.yaml.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz with the map-frame preset.'),
    ]

    # Chassis micro-ROS agent + robot_state_publisher (URDF + static TF).
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, 'launch', 'teleop.launch.py'])),
        launch_arguments={
            'serial_dev': serial_dev,
            'serial_baud': serial_baud,
        }.items(),
    )

    # UM982 RTK GNSS + NTRIP (lifecycle auto-activated inside).
    gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([um982, 'launch', 'gnss_rtk.launch.py'])),
        launch_arguments={
            'ntrip_params_file': ntrip_params_file,
        }.items(),
    )

    # Dual-EKF + navsat + confidence_gate + heading adapter.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [loc, 'launch', 'global_localization.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([bringup, 'config', 'gnss.rviz'])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [teleop, gnss, localization, rviz])
