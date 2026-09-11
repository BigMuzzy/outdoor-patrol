# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Expand real ROS launch descriptions without starting hardware processes."""

from dataclasses import dataclass, field
import os
from pathlib import Path
import tempfile

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    PopLaunchConfigurations,
    PushLaunchConfigurations,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState, StateTransition
from lifecycle_msgs.msg import State, Transition, TransitionEvent
import pytest
import yaml


SRC = Path(__file__).resolve().parents[2]
REPO = SRC.parent


@dataclass
class ExpandedNode:
    """Node arguments after ROS launch has resolved substitutions."""

    action: Node
    name: str
    package: str
    executable: str
    parameters: dict
    parameter_files: tuple
    remappings: tuple
    arguments: tuple
    delay: float


@dataclass
class ExpandedGraph:
    """Observed nodes and lifecycle requests, with no running processes."""

    nodes: dict = field(default_factory=dict)
    configure: set = field(default_factory=set)
    activate_on_inactive: set = field(default_factory=set)
    configurations: dict = field(default_factory=dict)


def _text(context, value):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value))


def _expand_node(action, context, delay):
    arguments = []
    for part in action.cmd[1:]:
        value = _text(context, part)
        if value == '--ros-args':
            break
        arguments.append(value)
    # Keep the private launch_ros parameter API isolated to this fixture.
    # It resolves real substitutions and writes the actual --params-file YAML.
    action._perform_substitutions(context)
    expanded_params = action._Node__expanded_parameter_arguments or []
    assert len(expanded_params) == len(action._Node__parameters or []), (
        f'{action.node_name} dropped a declared parameter file')
    name = action.node_name.replace('<node_namespace_unspecified>', '')
    parameters = {}
    files = []
    for path, is_file in expanded_params:
        assert is_file, 'Extend the fixture for command-line -p parameters.'
        files.append(Path(path))
        document = yaml.safe_load(Path(path).read_text())
        for selector in ('/**', name.lstrip('/'), name):
            parameters.update(document.get(selector, {}).get(
                'ros__parameters', {}))
    return ExpandedNode(
        action=action,
        name=name.lstrip('/'),
        package=_text(context, action.node_package),
        executable=_text(context, action.node_executable),
        parameters=parameters,
        parameter_files=tuple(files),
        remappings=tuple(action.expanded_remapping_rules or []),
        arguments=tuple(arguments),
        delay=delay,
    )


def _state_transition(action, goal):
    return StateTransition(action=action, msg=TransitionEvent(
        transition=Transition(label='transition_success'),
        start_state=State(label='configuring'),
        goal_state=State(label=goal),
    ))


@pytest.fixture
def launch_graph(tmp_path, monkeypatch):
    """Resolve package shares from source and walk launch with real contexts."""
    for role in ('GNSS', 'IMU', 'LIDAR'):
        for suffix in ('PORT', 'PARAMS'):
            monkeypatch.delenv(role + '_' + suffix, raising=False)
    prefix = tmp_path / 'prefix'
    markers = prefix / 'share/ament_index/resource_index/packages'
    markers.mkdir(parents=True)
    for package in SRC.iterdir():
        if (package / 'package.xml').is_file():
            (markers / package.name).touch()
            (prefix / 'share' / package.name).symlink_to(package)
    monkeypatch.setenv(
        'AMENT_PREFIX_PATH',
        str(prefix) + ':' + os.environ.get('AMENT_PREFIX_PATH', ''))
    monkeypatch.setenv('TMPDIR', str(tmp_path))
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))

    def expand(package='outdoor_patrol_bringup',
               launch_file='gnss_localization.launch.py', **overrides):
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        graph = ExpandedGraph()
        configure_events = []
        activation_events = []

        def walk(entities, delay=0.0):
            for entity in entities:
                condition = getattr(entity, 'condition', None)
                if condition is not None and not condition.evaluate(context):
                    continue
                if isinstance(entity, Node):
                    node = _expand_node(entity, context, delay)
                    name = node.name
                    assert name not in graph.nodes, f'Duplicate node: {name}'
                    graph.nodes[name] = node
                elif isinstance(entity, TimerAction):
                    seconds = perform_typed_substitution(
                        context, entity.period, float)
                    # TimerAction.handle restores its captured launch config
                    # and scopes its children; do not schedule a real timer.
                    PushLaunchConfigurations().execute(context)
                    walk(entity.actions, delay + seconds)
                    PopLaunchConfigurations().execute(context)
                elif isinstance(entity, RegisterEventHandler):
                    handler = entity.event_handler
                    assert isinstance(handler, OnStateTransition)
                    for child in handler.entities:
                        assert isinstance(child, EmitEvent)
                        assert isinstance(child.event, ChangeState)
                        assert child.event.transition_id == (
                            Transition.TRANSITION_ACTIVATE)
                        activation_events.append((handler, child.event))
                elif isinstance(entity, EmitEvent):
                    assert isinstance(entity.event, ChangeState)
                    assert entity.event.transition_id == (
                        Transition.TRANSITION_CONFIGURE)
                    configure_events.append(entity.event)
                elif isinstance(entity, LaunchDescription):
                    walk(entity.entities, delay)
                else:
                    assert not isinstance(entity, ExecuteProcess)
                    walk(entity.execute(context) or [], delay)

        path = SRC / package / 'launch' / launch_file
        description = PythonLaunchDescriptionSource(str(path))
        walk([description.get_launch_description(context)])

        def target_name(event):
            targets = [
                name for name, node in graph.nodes.items()
                if event.lifecycle_node_matcher(node.action)
            ]
            assert len(targets) == 1
            return targets[0]

        graph.configure.update(target_name(event) for event in configure_events)
        for handler, event in activation_events:
            target = target_name(event)
            for name, node in graph.nodes.items():
                if isinstance(node.action, LifecycleNode):
                    matches = handler.matches(
                        _state_transition(node.action, 'inactive'))
                    assert matches == (name == target), (
                        f'Activation must follow {target} entering inactive')
            assert not handler.matches(
                _state_transition(graph.nodes[target].action, 'active'))
            graph.activate_on_inactive.add(target)
        graph.configurations = dict(context.launch_configurations)
        return graph

    return expand
