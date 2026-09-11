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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
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
    imu_pkg = FindPackageShare('imu_driver')
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
            'gnss_dev', default_value='',
            description='Override the GNSS serial port; empty keeps its YAML.'),
        DeclareLaunchArgument(
            'imu_dev', default_value='',
            description='Override the IMU serial port; empty keeps its YAML.'),
        DeclareLaunchArgument(
            'um982_params_file',
            default_value=PathJoinSubstitution(
                [um982, 'config', 'um982_rover.yaml']),
            description='Legacy alias for gnss_params_file.'),
        DeclareLaunchArgument(
            'gnss_params_file',
            default_value=LaunchConfiguration('um982_params_file'),
            description='GNSS driver YAML; gnss_dev overrides its port.'),
        DeclareLaunchArgument(
            'imu_params_file',
            default_value=PathJoinSubstitution(
                [imu_pkg, 'config', 'imu_driver.yaml']),
            description='IMU driver YAML; imu_dev overrides its port.'),
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
            default_value=PathJoinSubstitution(
                [ntrip, 'config', 'ntrip.yaml.example']),
            description='NTRIP caster credentials YAML. Override with your '
                        'real ntrip.yaml.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz with the map-frame preset.'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Launch the M2 imu_driver and feed /imu_driver/data '
                        'to the global EKF as imu1 (yaw-rate).'),
        DeclareLaunchArgument(
            'imu_start_delay', default_value='12.0',
            description='Seconds to delay imu_driver startup so the micro-ROS '
                        'agent + local EKF latch onto /odom first — avoids '
                        'the M2 startup-race regression (2026-07-04).'),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Launch the M3 RPLIDAR C1 (2D) + scan_safety forward '
                        'brake (ADR-013).'),
        DeclareLaunchArgument(
            'lidar_dev',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'f86253bee863ef11a2a1e2a9c169b110-if00-port0'),
            description='Serial by-id path of the RPLIDAR C1 (CP2102N). '
                        'Override with lidar_dev:=... .'),
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

    # UM982 RTK GNSS + NTRIP (lifecycle auto-activated inside).
    # Driver-private params_file/port/auto_activate must not inherit or leak
    # through sibling includes. Group launch configurations resolve in parent.
    gnss = GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            'um982_params_file': LaunchConfiguration('gnss_params_file'),
            'port': LaunchConfiguration('gnss_dev'),
            'ntrip_params_file': ntrip_params_file,
            'rtcm_topic': LaunchConfiguration('rtcm_topic'),
            'auto_activate': LaunchConfiguration('gnss_auto_activate'),
            'ros_namespace': LaunchConfiguration('ros_namespace', default=''),
            'use_sim_time': use_sim_time,
        },
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([um982, 'launch', 'gnss_rtk.launch.py'])),
        )],
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
            'params_file': LaunchConfiguration('imu_params_file'),
            'port': LaunchConfiguration('imu_dev'),
            'baud': LaunchConfiguration('imu_baud'),
            'auto_activate': LaunchConfiguration('imu_auto_activate'),
            'ros_namespace': LaunchConfiguration('ros_namespace', default=''),
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
            'use_static_tf': 'false',
        },
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [imu_pkg, 'launch', 'imu_driver.launch.py'])),
        )],
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
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_dev'),
            'serial_baudrate': 460800,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('scan', '/scan_raw')],
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
        actions=[lidar_node, scan_filter, scan_safety],
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
