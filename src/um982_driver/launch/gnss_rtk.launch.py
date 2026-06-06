# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Combined RTK GNSS bringup: UM982 rover + NTRIP correction source.

Wires the full VRS correction loop so a single command brings the rover to
an RTK fix:

    ntrip_client.rtcm/out ---------> um982_driver.rtcm/in   (corrections in)
    um982_driver.~/nmea_sentence --> ntrip_client.nmea_sentence (GGA upload)

The UM982 publishes in its private namespace, so its raw GGA lands on
``/um982_driver/nmea_sentence``; the NTRIP client subscribes there to feed
VRS / nearest-base mountpoints.

Alternative correction sources: instead of ``ntrip_client_node`` you can run
``tcp_rtcm_relay`` or ``serial_rtcm_relay`` (from the ntrip_client package)
publishing to the same ``rtcm_topic`` — the UM982 side is unchanged.

Usage:

    ros2 launch um982_driver gnss_rtk.launch.py \\
        ntrip_params_file:=/path/to/ntrip.yaml \\
        um982_params_file:=/path/to/um982_rover.yaml
"""
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare

# The UM982 publishes private topics under its node name; keep the name and
# the GGA remap target in lock-step.
UM982_NODE_NAME = 'um982_driver'
NMEA_TOPIC = '/' + UM982_NODE_NAME + '/nmea_sentence'


def generate_launch_description() -> LaunchDescription:
    um982_pkg = FindPackageShare('um982_driver')
    ntrip_pkg = FindPackageShare('ntrip_client')

    default_um982 = PathJoinSubstitution(
        [um982_pkg, 'config', 'um982_rover.yaml'])
    default_ntrip = PathJoinSubstitution(
        [ntrip_pkg, 'config', 'ntrip.yaml.example'])

    um982_params_arg = DeclareLaunchArgument(
        'um982_params_file',
        default_value=default_um982,
        description='Path to UM982 rover parameter YAML.',
    )
    ntrip_params_arg = DeclareLaunchArgument(
        'ntrip_params_file',
        default_value=default_ntrip,
        description='Path to NTRIP caster credentials YAML.',
    )
    rtcm_topic_arg = DeclareLaunchArgument(
        'rtcm_topic',
        default_value='/rtcm',
        description='Shared topic carrying rtcm_msgs/Message corrections.',
    )
    auto_activate_arg = DeclareLaunchArgument(
        'auto_activate',
        default_value='true',
        description='Configure+activate the UM982 lifecycle node on launch.',
    )

    rtcm_topic = LaunchConfiguration('rtcm_topic')

    ntrip_client = Node(
        package='ntrip_client',
        executable='ntrip_client_node',
        name='ntrip_client',
        output='screen',
        parameters=[LaunchConfiguration('ntrip_params_file')],
        remappings=[
            ('rtcm/out', rtcm_topic),
            ('nmea_sentence', NMEA_TOPIC),
        ],
    )

    um982 = LifecycleNode(
        package='um982_driver',
        executable='um982_driver_node',
        name=UM982_NODE_NAME,
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('um982_params_file')],
        remappings=[
            ('rtcm/in', rtcm_topic),
        ],
    )

    # auto_activate: drive the lifecycle node unconfigured -> inactive -> active.
    configure_um982 = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(um982),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=um982,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(um982),
                        transition_id=(
                            lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE),
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    return LaunchDescription([
        um982_params_arg,
        ntrip_params_arg,
        rtcm_topic_arg,
        auto_activate_arg,
        ntrip_client,
        um982,
        activate_on_inactive,
        configure_um982,
    ])
