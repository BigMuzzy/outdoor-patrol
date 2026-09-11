# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Opt-in test of real Compose rendering, ROS argv parsing and node expansion."""

import os
from pathlib import Path

import pytest
from ros2launch.api.api import parse_launch_arguments
from test_hardware_launch import DEFAULT_NODES
import yaml


REPO = Path(__file__).resolve().parents[3]
ALTERNATE = Path(__file__).resolve().parent / 'fixtures/alternate_sensor.launch.py'
CONFIGS = {
    'gnss': ('um982_driver', 'port', 'baudrate', 38400),
    'imu': ('imu_driver', 'port', 'baudrate', 921600),
    'lidar': ('sllidar_node', 'serial_port', 'serial_baudrate', 256000),
}
pytestmark = pytest.mark.skipif(
    not os.environ.get('OUTDOOR_PATROL_COMPOSE_BIN'),
    reason='Needs standalone Compose v2 and the deployment install layout; '
           'see deploy/README.md integration-test instructions.',
)


@pytest.fixture
def compose_service(monkeypatch):
    """Reuse the real Compose renderer without needing a Docker daemon."""
    monkeypatch.syspath_prepend(str(REPO))
    from deploy.test.test_compose_devices import render_compose
    return render_compose


@pytest.mark.parametrize('filename', [
    'docker-compose.yaml', 'docker-compose.nav2.yaml',
])
@pytest.mark.parametrize('case', [
    'default', 'gnss-params', 'imu-params', 'lidar-params', 'network-lidar',
])
def test_compose_command_reaches_drivers(
        launch_graph, compose_service, monkeypatch, tmp_path, filename, case):
    variables = {}
    overlays = ()
    if case.endswith('-params'):
        role = case.removesuffix('-params')
        name, port_key, baud_key, baud = CONFIGS[role]
        config = tmp_path / (role + '.yaml')
        config.write_text(yaml.safe_dump({
            name: {'ros__parameters': {
                port_key: '/dev/from-yaml', baud_key: baud,
            }},
        }))
        variables[role.upper() + '_PARAMS'] = str(config)
    elif case == 'network-lidar':
        variables = {
            'LIDAR_PORT': '',
            'LIDAR_LAUNCH_FILE': str(ALTERNATE),
        }
        overlays = ('docker-compose.network-lidar.yaml',)
    robot = compose_service(filename, variables, override_files=overlays)
    assert robot['command'][:4] == [
        'ros2', 'launch', 'outdoor_patrol_bringup', 'gnss_localization.launch.py',
    ]
    arguments = dict(parse_launch_arguments(robot['command'][4:]))
    for name, value in robot['environment'].items():
        assert isinstance(value, str)
        monkeypatch.setenv(name, value)
    graph = launch_graph(**arguments)
    expected = DEFAULT_NODES
    if case == 'network-lidar':
        expected = expected - {'sllidar_node'} | {'replacement_lidar'}
        assert graph.nodes['replacement_lidar'].parameters['device_uri'] == (
            'tcp://192.0.2.10:1234')
        assert graph.nodes['replacement_lidar'].remappings == (
            ('laser', '/scan_raw'),)
    assert set(graph.nodes) == expected
    for role, (name, port_key, baud_key, baud) in CONFIGS.items():
        if role == 'lidar' and case == 'network-lidar':
            continue
        assert graph.nodes[name].parameters[port_key] == '/dev/op-' + role
        if case == role + '-params':
            assert graph.nodes[name].parameters[baud_key] == baud
    assert graph.nodes['micro_ros_agent'].arguments[2] == '/dev/op-chassis'
    assert graph.nodes['scan_safety'].remappings == (
        ('scan', '/scan'),
        ('cmd_vel_in', '/cmd_vel_raw'),
        ('cmd_vel_out', '/cmd_vel'),
    )
