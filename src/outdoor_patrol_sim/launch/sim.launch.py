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

Drive it from a second terminal (needs a real TTY). The M3 forward brake is
in the command path by default, so teleop must publish to `/cmd_vel_raw`::

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r /cmd_vel:=/cmd_vel_raw

Publishing straight to `/cmd_vel` reaches the chassis WITHOUT passing the
brake -- in sim exactly as it would on the robot. Pass `safety:=false` if you
deliberately want the ungated path.

Note: with `localization:=true` the included `heading_to_imu` node starts but
stays silent -- its UM982 input topic does not exist in sim, and `gnss_sim`
publishes `/gnss/heading` directly from ground truth instead.

Caveat: the gz DiffDrive plugin has NO command watchdog of its own -- it
holds the last `/cmd_vel` forever. What stops the robot in sim is
`scan_safety`, which holds the last raw command, re-gates it against the
freshest scan at 20 Hz, and emits a zero Twist once the command goes
unrefreshed for `cmd_timeout_s` (0.5 s, mirroring the firmware's
CMD_VEL_TIMEOUT_MS). So a single keypress drives for ~0.5 s and stops, as it
does on the robot. Two consequences: with `safety:=false` nothing stops the
robot at all, and the sim still cannot validate the firmware's own
stop-on-signal-loss path -- only the brake's stand-in for it.

Only run ONE instance at a time. Two `gz sim` servers share the same
gz-transport topic names, so a second sim silently steals `/cmd_vel` and
`/scan` from the first and nothing behaves as expected.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
            'safety', default_value='true',
            description='Run the M3 scan_safety forward brake between '
                        '/cmd_vel_raw and /cmd_vel. On by default because it '
                        'is always in the path on the robot. NOTE: you must '
                        'then drive via /cmd_vel_raw -- publishing straight '
                        'to /cmd_vel bypasses the brake, in sim exactly as it '
                        'would on the robot.'),
        DeclareLaunchArgument(
            'localization_start_delay', default_value='3.0',
            description='Extra seconds to wait after /clock goes live before '
                        'starting the EKF stack. The launch already GATES on '
                        'the first /clock message (see clock_gate below); '
                        'this is only margin on top of that.'),
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
        DeclareLaunchArgument(
            'datum_params_file',
            default_value=PathJoinSubstitution(
                [sim_pkg, 'config', 'navsat_datum_sim.yaml']),
            description='navsat_transform datum overlay. Defaults to the '
                        'FIXED sim datum, which pins the map frame to the '
                        'Gazebo world origin so /odometry/global can be '
                        'compared against /odom_truth and so a recorded '
                        'route replays in the next run. Point at '
                        'outdoor_patrol_loc/config/datum_auto.yaml for the '
                        'robot default (auto-datum on first fix).'),
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

    # A robot_localization node calls Clock::wait_until_started() inside
    # initialize(), BEFORE its executor spins. If /clock is not already live
    # when it gets there it parks on "Waiting for clock to start..." and never
    # recovers -- no odom -> base_link, no map -> odom, and the EKF topics stay
    # silent. A fixed delay does not fix this reliably (how long Gazebo takes
    # to load the world and start stepping varies with the machine and with
    # how many sensors the world has), so gate on the real event: block until
    # /clock actually delivers a message, then start the stack.
    clock_gate = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/clock', '--once'],
        output='log',
        condition=IfCondition(use_localization),
    )

    localization = RegisterEventHandler(
        OnProcessExit(
            target_action=clock_gate,
            on_exit=[TimerAction(
                period=LaunchConfiguration('localization_start_delay'),
                actions=[GroupAction(
                    scoped=True,
                    actions=[IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [loc_pkg, 'launch',
                                 'global_localization.launch.py'])),
                        launch_arguments={
                            'use_sim_time': 'true',
                            'datum_params_file': LaunchConfiguration(
                                'datum_params_file'),
                        }.items(),
                    )],
                )],
            )],
        ),
        condition=IfCondition(use_localization),
    )

    # M3 forward brake. Runs against the SAME raw-scan convention as the
    # robot: scan_safety reads raw angles (no TF), and the sim's gpu_lidar
    # hangs off the yaw-pi lidar_link, so forward_offset_deg=180 is correct
    # here for the same reason it is on the real C1.
    # NOTE: the includes here are wrapped in SCOPED GroupActions.
    # IncludeLaunchDescription does NOT scope by default, so a
    # DeclareLaunchArgument inside an included file leaks its value into this
    # context. scan_safety.launch.py and localization.launch.py BOTH declare
    # an argument called `params_file`; unscoped, the safety include leaks
    # scan_safety.yaml, and the EKF -- included later -- sees `params_file`
    # already set, skips its own default, and silently loads the safety YAML
    # instead of ekf.yaml. It then comes up with no odom0 input and publishes
    # nothing at all. Do not remove the scoping.
    safety = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [safety_pkg, 'launch', 'scan_safety.launch.py'])),
            launch_arguments={'use_sim_time': 'true'}.items(),
        )],
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
        clock_gate,
        localization,
        rviz,
    ])
