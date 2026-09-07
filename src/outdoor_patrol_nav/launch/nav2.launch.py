# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Nav2 stack for the patrol robot — Phase 1 of the Nav2 migration.

Brings up planner, controller, smoother, behaviours, bt_navigator, velocity
smoother and the lifecycle manager that sequences them. It does NOT start
`patrol_mission`: the mission node takes a `route_path` that only the caller
knows, so `run_validation.sh` starts it separately, exactly as it starts
`route_follower` today.

Also NOT started: `map_server` and the costmap filters. There is no occupancy
map of the site — GNSS is the map — and the corridor keepout mask arrives with
`route_to_map` in Phase 2.

Velocity path (the part that is easy to break):

    controller_server ─┐
                       ├─► /cmd_vel_nav ─► velocity_smoother ─► /cmd_vel_raw
    behavior_server  ──┘                                              │
                                                                      ▼
                                           scan_safety (ADR-013) ─► /cmd_vel

`/cmd_vel` is scan_safety's OUTPUT. Every Nav2 server that publishes velocity
defaults to `cmd_vel`, so each one is remapped below. Miss one and there are
two writers on scan_safety's output topic — the brake still runs, but the
robot obeys whichever message arrived last.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Order matters: the lifecycle manager configures and activates these in
# sequence, and bt_navigator must come up after the servers it calls.
LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('outdoor_patrol_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Have the lifecycle manager configure and activate the '
                    'stack on start. Set false to step the transitions by '
                    'hand when debugging a configure-time parameter error.',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg, 'config',
                                            'nav2_params.yaml']),
        description='Nav2 parameter file for every server below.',
    )
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml',
        default_value=PathJoinSubstitution([pkg, 'bt', 'patrol.xml']),
        description='Behaviour tree bt_navigator runs for '
                    'NavigateThroughPoses. Defaults to the pinned copy of '
                    "Nav2's replanning-with-recovery tree plus SmoothPath.",
    )

    # A plain dict here becomes a wildcard (`/**`) parameter overlay, which is
    # what makes it reach the costmap nodes too: local_costmap and
    # global_costmap are separate nodes living inside the controller and
    # planner processes, and a per-node override would miss them.
    common = {'use_sim_time': use_sim_time}

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, common],
        remappings=[('cmd_vel', '/cmd_vel_nav')],
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, common],
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, common],
    )

    # Spin / BackUp / DriveOnHeading / Wait. They drive the robot themselves,
    # so they need the same remap as the controller.
    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, common],
        remappings=[('cmd_vel', '/cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            common,
            {'default_nav_through_poses_bt_xml': bt_xml,
             'default_nav_to_pose_bt_xml': bt_xml},
        ],
    )

    # The only Nav2 node that writes to the robot. Its output is
    # `cmd_vel_smoothed`; sending that to /cmd_vel_raw keeps scan_safety
    # between Nav2 and the wheels, which is the whole point of ADR-013.
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, common],
        remappings=[('cmd_vel', '/cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/cmd_vel_raw')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            common,
            {'autostart': autostart, 'node_names': LIFECYCLE_NODES},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        params_file_arg,
        bt_xml_arg,
        controller,
        smoother,
        planner,
        behaviors,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
    ])
