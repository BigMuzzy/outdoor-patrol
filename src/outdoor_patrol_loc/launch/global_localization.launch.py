# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Interim GNSS global localization (ADR-012) — dual-EKF + navsat_transform.

Composes on top of the M1 local EKF (localization.launch.py):

  - local EKF       : wheel velocities -> `odom -> base_link` (smooth)
  - heading_to_imu  : UM982 dual-antenna heading -> /gnss/heading (Imu, yaw)
  - confidence_gate : raw NavSatFix -> /gnss/fix_gated (inflate cov on
                      degraded fix; drop NO_FIX)
  - navsat_transform: gated NavSatFix + heading -> /odometry/gps
  - global EKF      : wheel + GNSS position + heading -> `map -> odom`

Not started here (run separately): the UM982 driver + NTRIP. `fix_topic`
defaults to the gated fix; override to the raw driver fix to bypass the gate.

TBD until measured / decided: the heading `yaw_offset` (heading_to_imu params)
- integration plan items 2/3. Datum defaults to auto-on-first-fix
(config/datum_auto.yaml); pass `datum_params_file:=<file>` to pin a fixed
per-site origin instead, which is what makes recorded routes comparable
across sessions.
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
    datum_params_file = LaunchConfiguration('datum_params_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    fix_topic_arg = DeclareLaunchArgument(
        'fix_topic',
        default_value='/gnss/fix_gated',
        description='NavSatFix input for navsat_transform. Defaults to the '
                    'confidence_gate output; set to /um982_driver/fix to '
                    'bypass the gate.',
    )
    datum_params_arg = DeclareLaunchArgument(
        'datum_params_file',
        default_value=PathJoinSubstitution([pkg, 'config',
                                            'datum_auto.yaml']),
        description='Datum-policy overlay layered on top of navsat.yaml. The '
                    'default auto-sets the map origin on the first fix; point '
                    'this at a file with wait_for_datum + datum to pin the '
                    'origin to a fixed per-site value, which is what makes '
                    'saved routes comparable across sessions.',
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
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'heading_to_imu.yaml']),
            {'use_sim_time': use_sim_time},
        ],
    )

    # Inflate covariance on a degraded fix; publish /gnss/fix_gated.
    confidence_gate = Node(
        package='outdoor_patrol_loc',
        executable='confidence_gate',
        name='confidence_gate',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'confidence_gate.yaml']),
            {'use_sim_time': use_sim_time},
        ],
    )

    # GNSS <-> map-frame bridge.
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'navsat.yaml']),
            datum_params_file,
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
        datum_params_arg,
        local_ekf,
        heading_to_imu,
        confidence_gate,
        navsat,
        global_ekf,
    ])
