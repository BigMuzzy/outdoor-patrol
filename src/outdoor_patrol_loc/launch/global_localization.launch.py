# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Interim GNSS global localization (ADR-012) — dual-EKF + navsat_transform.

Composes on top of the M1 local EKF (localization.launch.py):

  - local EKF       : wheel velocities -> `odom -> base_link` (smooth)
  - heading_to_imu  : UM982 dual-antenna heading -> /gnss/heading (Imu, yaw)
  - navsat_transform: gated NavSatFix + heading -> /odometry/gps
  - global EKF      : wheel + GNSS position + heading -> `map -> odom`

Not started here (run separately): the UM982 driver + NTRIP, and the
confidence_gate (step 3). Point `fix_topic` at the gated fix once the gate
lands; until then it defaults to the raw driver fix.

TBD until measured / decided: the heading `yaw_offset` (heading_to_imu params)
and the map datum (config/navsat.yaml) — integration plan items 2/3/4.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('outdoor_patrol_loc')

    use_sim_time = LaunchConfiguration('use_sim_time')
    fix_topic = LaunchConfiguration('fix_topic')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    fix_topic_arg = DeclareLaunchArgument(
        'fix_topic',
        default_value='/um982_driver/fix',
        description='NavSatFix input for navsat_transform. Point at the '
                    'confidence_gate output (e.g. /gnss/fix_gated) once '
                    'step 3 lands.',
    )

    # M1 local EKF (odom -> base_link), reused unchanged.
    local_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'localization.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Dual-antenna heading (QuaternionStamped) -> yaw-only Imu.
    heading_to_imu = Node(
        package='outdoor_patrol_loc',
        executable='heading_to_imu',
        name='heading_to_imu',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # GNSS <-> map-frame bridge.
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'navsat.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('gps/fix', fix_topic),
            ('imu', '/gnss/heading'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    # Global EKF: owns map -> odom.
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'ekf_global.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        fix_topic_arg,
        local_ekf,
        heading_to_imu,
        navsat,
        global_ekf,
    ])
