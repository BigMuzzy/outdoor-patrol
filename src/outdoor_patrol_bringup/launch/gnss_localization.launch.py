# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""
Interim GNSS global-localization bringup (ADR-012) — full stack, one launch.

Composes everything needed to run the interim global localization on the real
robot:

  teleop.launch.py              micro-ROS agent + robot_state_publisher
   [outdoor_patrol_bringup]     -> /odom, /cmd_vel, TF base_link<->gnss_link
  profiles/gnss_um982.launch.py UM982 driver + NTRIP (RTK), lifecycle
   [outdoor_patrol_bringup]     auto-activated -> /um982_driver/fix, /heading
  global_localization.launch.py dual-EKF + heading adapter + confidence_gate +
   [outdoor_patrol_loc]         navsat_transform -> odom->base_link, map->odom
  rviz2 (optional)              fixed frame = map

Drive with keyboard teleop in a SEPARATE terminal (needs a real TTY).
Remap its /cmd_vel publisher to /cmd_vel_raw to keep the lidar brake in the
command path; see the repository README (GNSS global localization) for the
full command.

NTRIP credentials: pass `ntrip_params_file:=/path/to/ntrip.yaml`; the default
points at the package example (no real caster).

Each sensor can select a different absolute *_launch_file path. The profile
owns driver/protocol adaptation, not localization, TF or the lidar brake.
Empty *_params_file and *_dev values retain the selected profile's defaults.
Recheck mount and heading/brake calibration when replacing hardware.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include_profile(context):
    role = LaunchConfiguration('sensor_role').perform(context)
    path = Path(LaunchConfiguration('driver_launch_file').perform(context))
    if not path.is_absolute():
        raise ValueError(f'{role}_launch_file must be an absolute path: {path}')
    if not path.is_file():
        raise FileNotFoundError(f'{role} launch profile not found: {path}')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))]


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare('outdoor_patrol_bringup')
    loc = FindPackageShare('outdoor_patrol_loc')
    safety_pkg = FindPackageShare('outdoor_patrol_safety')

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
            'gnss_dev',
            default_value=EnvironmentVariable('GNSS_PORT', default_value=''),
            description='Override the GNSS serial port; empty keeps its YAML.'),
        DeclareLaunchArgument(
            'imu_dev',
            default_value=EnvironmentVariable('IMU_PORT', default_value=''),
            description='Override the IMU serial port; empty keeps its YAML.'),
        DeclareLaunchArgument(
            'um982_params_file',
            default_value=EnvironmentVariable('GNSS_PARAMS', default_value=''),
            description='Legacy alias for gnss_params_file.'),
        DeclareLaunchArgument(
            'gnss_params_file',
            default_value=LaunchConfiguration('um982_params_file'),
            description='GNSS driver YAML; empty uses the selected profile.'),
        DeclareLaunchArgument(
            'imu_params_file',
            default_value=EnvironmentVariable('IMU_PARAMS', default_value=''),
            description='IMU driver YAML; empty uses the selected profile.'),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=EnvironmentVariable('LIDAR_PARAMS', default_value=''),
            description='LiDAR driver YAML; empty uses the selected profile.'),
        DeclareLaunchArgument(
            'gnss_launch_file',
            default_value=PathJoinSubstitution([
                bringup, 'launch', 'profiles', 'gnss_um982.launch.py']),
            description='Absolute GNSS profile launch path.'),
        DeclareLaunchArgument(
            'imu_launch_file',
            default_value=PathJoinSubstitution([
                bringup, 'launch', 'profiles', 'imu_inertial_labs.launch.py']),
            description='Absolute IMU profile launch path.'),
        DeclareLaunchArgument(
            'lidar_launch_file',
            default_value=PathJoinSubstitution([
                bringup, 'launch', 'profiles', 'lidar_rplidar_c1.launch.py']),
            description='Absolute LiDAR profile launch path.'),
        DeclareLaunchArgument(
            'gnss_auto_activate',
            default_value=LaunchConfiguration('auto_activate', default='true'),
            description='Activate GNSS on launch. Legacy auto_activate now '
                        'controls GNSS only, never the IMU.'),
        DeclareLaunchArgument(
            'imu_auto_activate', default_value='true',
            description='Activate the IMU independently of GNSS.'),
        DeclareLaunchArgument(
            'imu_baud',
            default_value=LaunchConfiguration('baud', default=''),
            description='Override IMU baud; empty keeps its YAML.'),
        DeclareLaunchArgument(
            'rtcm_topic', default_value='/rtcm',
            description='RTCM correction topic shared by GNSS and NTRIP.'),
        DeclareLaunchArgument(
            'ntrip_params_file',
            default_value='',
            description='NTRIP credentials YAML; empty uses the GNSS profile '
                        'default (stock: example with no real caster).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz with the map-frame preset.'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Launch the IMU profile and feed /imu_driver/data '
                        'to the global EKF as imu1 (yaw-rate).'),
        DeclareLaunchArgument(
            'imu_start_delay', default_value='12.0',
            description='Seconds to delay imu_driver startup so the micro-ROS '
                        'agent + local EKF latch onto /odom first — avoids '
                        'the M2 startup-race regression (2026-07-04).'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Launch the LiDAR profile + scan_safety forward '
                        'brake (ADR-013).'),
        DeclareLaunchArgument(
            'lidar_dev',
            default_value=EnvironmentVariable('LIDAR_PORT', default_value=''),
            description='Override LiDAR port; empty keeps profile settings.'),
        DeclareLaunchArgument(
            'lidar_start_delay', default_value='8.0',
            description='Seconds to delay the LiDAR driver + safety node so '
                        'core discovery settles first (as imu_start_delay).'),
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

    # GNSS profile (stock: UM982 RTK GNSS + NTRIP, lifecycle auto-activated).
    # Driver-private params_file/port/auto_activate must not inherit or leak
    # through sibling includes. Group launch configurations resolve in parent.
    gnss = GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            'sensor_role': 'gnss',
            'driver_launch_file': LaunchConfiguration('gnss_launch_file'),
            'params_file': LaunchConfiguration('gnss_params_file'),
            'port': LaunchConfiguration('gnss_dev'),
            'ntrip_params_file': ntrip_params_file,
            'rtcm_topic': LaunchConfiguration('rtcm_topic'),
            'auto_activate': LaunchConfiguration('gnss_auto_activate'),
            'ros_namespace': LaunchConfiguration('ros_namespace', default=''),
            'use_sim_time': use_sim_time,
        },
        actions=[OpaqueFunction(function=_include_profile)],
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

    # M2 IMU (imu_driver) -> /imu_driver/data, fused as imu1 (yaw-rate) in the
    # global EKF. DELAYED START: bringing it up with the rest of the stack
    # shifts DDS discovery timing so the local EKF (ekf_filter_node) races the
    # micro-ROS /odom and never latches (M2 regression, 2026-07-04). The
    # TimerAction starts it only AFTER the micro-ROS agent + EKFs are up and
    # /odom flows. use_static_tf:=false -> robot_state_publisher owns
    # base_link -> imu_link (from the URDF).
    imu = GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            'sensor_role': 'imu',
            'driver_launch_file': LaunchConfiguration('imu_launch_file'),
            'params_file': LaunchConfiguration('imu_params_file'),
            'port': LaunchConfiguration('imu_dev'),
            'baud': LaunchConfiguration('imu_baud'),
            'auto_activate': LaunchConfiguration('imu_auto_activate'),
            'ros_namespace': LaunchConfiguration('ros_namespace', default=''),
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
            'use_static_tf': 'false',
        },
        actions=[OpaqueFunction(function=_include_profile)],
    )
    imu_delayed = TimerAction(
        period=LaunchConfiguration('imu_start_delay'),
        actions=[imu],
        condition=IfCondition(LaunchConfiguration('use_imu')),
    )

    # M3 (ADR-013): 2D RPLIDAR C1 obstacle avoidance. Pipeline:
    #   sllidar_node -> /scan_raw -> scan_box_filter (laser_filters
    #   LaserScanBoxFilter: drops returns that land inside the robot's own
    #   body -- a footprint box in base_link derived from config/chassis.yaml,
    #   masking the chassis this nose-mounted unit sees around itself)
    #   -> /scan -> scan_safety (forward brake) + costmap / RViz.
    # Mount: lidar_link yawed pi (chassis.yaml) + inverted:=FALSE gives correct
    # fwd/back AND L/R (confirmed in RViz 2026-07-11). scan_safety gates
    # /cmd_vel_raw -> /cmd_vel (forward-only; reverse + rotation pass) using
    # forward_offset_deg=180. Delayed like the IMU so core discovery settles
    # first; nothing here writes /odom or map->odom. scan_safety is inert until
    # something publishes /cmd_vel_raw (drive teleop/Nav2 to /cmd_vel_raw); it
    # then HOLDS that command and re-gates it at 20 Hz until the command goes
    # unrefreshed for cmd_timeout_s, so it can also stop a robot that is
    # already moving.
    lidar = GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            'sensor_role': 'lidar',
            'driver_launch_file': LaunchConfiguration('lidar_launch_file'),
            'params_file': LaunchConfiguration('lidar_params_file'),
            'port': LaunchConfiguration('lidar_dev'),
            'ros_namespace': LaunchConfiguration('ros_namespace', default=''),
            'use_sim_time': use_sim_time,
        },
        actions=[OpaqueFunction(function=_include_profile)],
    )
    # laser_filters LaserScanBoxFilter: masks the robot's own body out of the
    # scan by dropping every return whose (x, y, z) in base_link falls inside
    # the footprint box (config/scan_box_filter.yaml, derived from chassis.yaml
    # body size). Needs base_link -> lidar_link TF from robot_state_publisher.
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_box_filter',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [bringup, 'config', 'scan_box_filter.yaml']),
        ],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan'),
        ],
    )
    scan_safety = Node(
        package='outdoor_patrol_safety',
        executable='scan_safety',
        name='scan_safety',
        output='screen',
        parameters=[
            PathJoinSubstitution([safety_pkg, 'config', 'scan_safety.yaml']),
        ],
        remappings=[
            ('scan', '/scan'),
            ('cmd_vel_in', '/cmd_vel_raw'),
            ('cmd_vel_out', '/cmd_vel'),
        ],
    )
    lidar_delayed = TimerAction(
        period=LaunchConfiguration('lidar_start_delay'),
        actions=[lidar, scan_filter, scan_safety],
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([bringup, 'config', 'gnss.rviz'])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        args + [teleop, gnss, localization, imu_delayed, lidar_delayed, rviz])
