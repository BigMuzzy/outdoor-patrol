# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Gazebo Harmonic simulation of the outdoor patrol robot.

Stands in for the whole physical stack — ESP32-S3 chassis controller, UM982
GNSS, UMKA IMU, RPLIDAR C1 — publishing on the SAME ROS topics the real
drivers use, so `outdoor_patrol_loc` runs against it unmodified::

    /cmd_vel  /odom  /joint_states  /scan  /imu_driver/data  /um982_driver/fix

Plus two sim-only extras: `/odom_truth` (ground truth, for scoring the EKF —
never fuse it) and `/clock` (every node here runs with `use_sim_time:=true`).

Headless by default so it works over SSH and in the dev container; pass
`gui:=true` for the Gazebo GUI on a machine with a display.

Drive it from a second terminal (needs a real TTY)::

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Note: with `localization:=true` the included `heading_to_imu` node starts but
stays silent — its UM982 input topic does not exist in sim, and `gnss_sim`
publishes `/gnss/heading` directly from ground truth instead.

Caveat: the gz DiffDrive plugin has NO command watchdog, so the sim keeps
driving on the last `/cmd_vel` forever. The real firmware fails safe and
stops. Do not use the sim to validate stop-on-signal-loss behaviour.

With `safety:=true` the M3 forward brake is spliced in ahead of the chassis,
so you must drive via `/cmd_vel_raw` or you bypass it::

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r /cmd_vel:=/cmd_vel_raw
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml


def _spawn_height() -> str:
    """Drop height for base_link: chassis ride height + 2 cm of clearance.

    Spawning base_link at ground level would bury the wheels by the full ride
    height and the solver would fling the robot into the air on the first
    step.
    """
    chassis_yaml = os.path.join(
        get_package_share_directory('outdoor_patrol_bringup'),
        'config', 'chassis.yaml')
    with open(chassis_yaml) as handle:
        chassis = yaml.safe_load(handle)['chassis']
    return str(chassis['base_height'] + 0.02)


def generate_launch_description() -> LaunchDescription:
    sim_pkg = FindPackageShare('outdoor_patrol_sim')
    loc_pkg = FindPackageShare('outdoor_patrol_loc')
    safety_pkg = FindPackageShare('outdoor_patrol_safety')
    sim_share = get_package_share_directory('outdoor_patrol_sim')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_localization = LaunchConfiguration('localization')
    use_gnss = LaunchConfiguration('gnss')
    use_safety = LaunchConfiguration('safety')

    args = [
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [sim_pkg, 'worlds', 'patrol_yard.sdf']),
            description='Absolute path to the SDF world to load.'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Show the Gazebo GUI. False runs headless (server '
                        'only, still renders the LiDAR offscreen).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz with the odometry preset.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [sim_pkg, 'config', 'sim.rviz']),
            description='RViz config used when use_rviz:=true. The default '
                        'preset is map-framed and overlays ground truth on '
                        'the global EKF estimate.'),
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Run the dual-EKF + navsat stack (map -> odom -> '
                        'base_link). False leaves TF to you.'),
        DeclareLaunchArgument(
            'gnss', default_value='true',
            description='Run gnss_sim (covariance-stamped /um982_driver/fix '
                        'and synthetic /gnss/heading).'),
        DeclareLaunchArgument(
            'safety', default_value='false',
            description='Run the M3 scan_safety forward brake between '
                        '/cmd_vel_raw and /cmd_vel. When true you MUST drive '
                        'via /cmd_vel_raw, or you bypass the brake: '
                        'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
                        '--ros-args -r /cmd_vel:=/cmd_vel_raw'),
        DeclareLaunchArgument(
            'x', default_value='0.0', description='Spawn X in the world.'),
        DeclareLaunchArgument(
            'y', default_value='0.0', description='Spawn Y in the world.'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Spawn yaw (rad) in the world.'),
        DeclareLaunchArgument(
            'z', default_value=_spawn_height(),
            description='Spawn height of base_link. Defaults to the chassis '
                        'ride height plus 2 cm so the wheels settle onto the '
                        'ground instead of starting inside it.'),
        DeclareLaunchArgument(
            'software_rendering', default_value='false',
            description='Force Mesa software (llvmpipe) rendering. Only '
                        'needed on a host with no /dev/dri render node; when '
                        'one exists, forcing it makes the headless EGL '
                        'context abort.'),
    ]

    # Set explicitly rather than inherited: this dev container ships
    # LIBGL_ALWAYS_SOFTWARE=1, which makes gz-sim's headless EGL context die
    # with "Not allowed to force software rendering when API explicitly
    # selects a hardware device" the moment the gpu_lidar starts rendering.
    gl_software = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE',
        PythonExpression([
            "'1' if '", LaunchConfiguration('software_rendering'),
            "'.lower() in ('true', '1') else '0'",
        ]),
    )

    # Lets Gazebo find package:// meshes and the world's own directory.
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(sim_share))

    # `-r` starts unpaused; `-s --headless-rendering` is the no-display path.
    gz_args = PythonExpression([
        "'", world, " -r -v 2' if '", gui,
        "'.lower() in ('true', '1') else '", world,
        " -r -v 2 -s --headless-rendering'",
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution(
                [sim_pkg, 'urdf', 'outdoor_patrol_sim.urdf.xacro']),
            ' sim:=true',
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    # Spawns the model straight from /robot_description, so the URDF above is
    # the single source of truth for both TF and physics.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_outdoor_patrol',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'outdoor_patrol',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(sim_share, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    gnss_sim = Node(
        package='outdoor_patrol_sim',
        executable='gnss_heading_sim',
        name='gnss_sim',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_gnss),
    )

    # Stamps the firmware's covariance onto the gz odometry: /odom_sim ->
    # /odom. Always on — without it the EKF trusts wheel odometry blindly.
    odom_sim = Node(
        package='outdoor_patrol_sim',
        executable='odom_sim',
        name='odom_sim',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [sim_pkg, 'config', 'odom_covariance.yaml']),
            {'use_sim_time': True},
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [loc_pkg, 'launch', 'global_localization.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_localization),
    )

    # M3 forward brake. Runs against the SAME raw-scan convention as the
    # robot: scan_safety reads raw angles (no TF), and the sim's gpu_lidar
    # hangs off the yaw-pi lidar_link, so forward_offset_deg=180 is correct
    # here for the same reason it is on the real C1.
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [safety_pkg, 'launch', 'scan_safety.launch.py'])),
        condition=IfCondition(use_safety),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [
        gl_software,
        resource_path,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        odom_sim,
        gnss_sim,
        safety,
        localization,
        rviz,
    ])
