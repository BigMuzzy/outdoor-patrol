# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Launch the Inertial Labs IMU driver with auto-activation, TF, and RViz."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


def _launch_setup(context, *args, **kwargs):
    """Build the driver plus optional auto-activation, static TF, and RViz."""
    params = [LaunchConfiguration('params_file')]
    baud = LaunchConfiguration('baud').perform(context)
    if baud:
        params.append({'baudrate': int(baud)})

    driver = LifecycleNode(
        package='imu_driver',
        executable='imu_driver_node',
        name='imu_driver',
        namespace='',
        output='screen',
        parameters=params,
    )
    actions = [driver]

    # Auto-configure on startup, then activate once the node reports 'inactive'.
    if LaunchConfiguration('auto_activate').perform(context).lower() == 'true':
        actions.append(EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )))
        actions.append(RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=driver,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(driver),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            ))],
        )))

    # Static transform so RViz has a valid fixed frame for the IMU data. The
    # child frame must match the driver's frame_id (default imu_link).
    parent_frame = LaunchConfiguration('parent_frame').perform(context)
    imu_frame = LaunchConfiguration('imu_frame').perform(context)
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_static_tf')),
        arguments=[
            '--frame-id', parent_frame,
            '--child-frame-id', imu_frame,
        ],
    ))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))

    return actions


def generate_launch_description() -> LaunchDescription:
    pkg = FindPackageShare('imu_driver')
    default_params = PathJoinSubstitution([pkg, 'config', 'imu_driver.yaml'])
    default_rviz = PathJoinSubstitution([pkg, 'rviz', 'imu.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the IMU driver parameter YAML.',
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='',
            description='Override the serial baudrate from params_file '
                        '(e.g. baud:=2000000). Empty keeps the YAML value.',
        ),
        DeclareLaunchArgument(
            'auto_activate',
            default_value='true',
            description='Auto-configure and activate the lifecycle node on '
                        'launch. Set false to drive the lifecycle manually.',
        ),
        DeclareLaunchArgument(
            'use_static_tf',
            default_value='true',
            description='Publish a static parent_frame->imu_frame transform.',
        ),
        DeclareLaunchArgument(
            'parent_frame',
            default_value='map',
            description='Parent frame for the static IMU transform.',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='imu_link',
            description='IMU child frame; must match the driver frame_id.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz preloaded with the IMU visualization.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config file to load.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
