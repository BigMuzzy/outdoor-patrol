# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Check stock and replacement profile boundaries using real launch expansion."""

from pathlib import Path

import pytest
from test_hardware_launch import DEFAULT_NODES
import yaml


ALTERNATE = Path(__file__).resolve().parent / 'fixtures/alternate_sensor.launch.py'
REPLACED = {
    'gnss': {'um982_driver', 'ntrip_client'},
    'imu': {'imu_driver'},
    'lidar': {'sllidar_node'},
}


@pytest.mark.parametrize('role', ['gnss', 'imu', 'lidar'])
def test_each_role_can_select_another_driver(launch_graph, role):
    graph = launch_graph(use_rviz='false', **{
        role + '_launch_file': str(ALTERNATE),
        role + '_dev': '/dev/replacement-' + role,
    })
    name = 'replacement_' + role
    assert set(graph.nodes) == DEFAULT_NODES - REPLACED[role] | {name}
    node = graph.nodes[name]
    assert node.package == 'test_vendor_driver'
    assert node.executable == 'test_sensor'
    assert node.parameters == {
        'model': 'replacement-test-fixture',
        'device_uri': '/dev/replacement-' + role,
        'frame_id': role + '_link',
    }
    assert node.delay == {'gnss': 0.0, 'imu': 12.0, 'lidar': 8.0}[role]
    assert 'sensor_role' not in graph.configurations
    assert 'driver_launch_file' not in graph.configurations


@pytest.mark.parametrize('role', ['gnss', 'imu', 'lidar'])
def test_empty_overrides_use_the_selected_profile_not_stock(launch_graph, role):
    graph = launch_graph(use_rviz='false', **{
        role + '_launch_file': str(ALTERNATE),
    })
    params = graph.nodes['replacement_' + role].parameters
    assert params['device_uri'] == 'tcp://192.0.2.10:1234'
    assert params['model'] == 'replacement-test-fixture'
    assert 'port' not in params
    assert 'baudrate' not in params


@pytest.mark.parametrize('role', ['gnss', 'imu', 'lidar'])
def test_replacement_params_then_explicit_port(launch_graph, tmp_path, role):
    config = tmp_path / 'replacement.yaml'
    config.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {
            'model': 'custom-config', 'device_uri': '/dev/from-config',
        }},
    }))
    graph = launch_graph(use_rviz='false', **{
        role + '_launch_file': str(ALTERNATE),
        role + '_params_file': str(config),
        role + '_dev': '/dev/explicit-override',
    })
    node = graph.nodes['replacement_' + role]
    assert node.parameters['model'] == 'custom-config'
    assert node.parameters['device_uri'] == '/dev/explicit-override'
    assert graph.nodes['ekf_filter_node'].parameters['frequency'] == 30.0


def test_replacement_lidar_keeps_filter_brake_and_tf(launch_graph):
    stock = launch_graph(use_rviz='false')
    changed = launch_graph(
        use_rviz='false', lidar_launch_file=str(ALTERNATE), lidar_dev='')
    assert set(changed.nodes) == (
        DEFAULT_NODES - {'sllidar_node'} | {'replacement_lidar'})
    assert changed.nodes['replacement_lidar'].remappings == (
        ('laser', '/scan_raw'),)
    for name in ('scan_box_filter', 'scan_safety', 'robot_state_publisher'):
        assert changed.nodes[name].parameters == stock.nodes[name].parameters
        assert changed.nodes[name].remappings == stock.nodes[name].remappings
        assert changed.nodes[name].delay == stock.nodes[name].delay


@pytest.mark.parametrize('role', ['gnss', 'imu', 'lidar'])
def test_missing_enabled_profile_fails(launch_graph, tmp_path, role):
    with pytest.raises(FileNotFoundError, match='launch profile not found'):
        launch_graph(**{
            role + '_launch_file': str(tmp_path / 'missing.launch.py'),
        })


@pytest.mark.parametrize('role', ['imu', 'lidar'])
def test_disabled_profiles_do_not_resolve_files(launch_graph, tmp_path, role):
    graph = launch_graph(use_rviz='false', **{
        'use_' + role: 'false',
        role + '_launch_file': str(tmp_path / 'absent.launch.py'),
        role + '_params_file': str(tmp_path / 'absent.yaml'),
    })
    omitted = REPLACED[role]
    if role == 'lidar':
        omitted = omitted | {'scan_box_filter', 'scan_safety'}
    assert set(graph.nodes) == DEFAULT_NODES - omitted


def test_profile_paths_are_not_cwd_dependent(launch_graph):
    with pytest.raises(ValueError, match='absolute path'):
        launch_graph(imu_launch_file='relative.launch.py')


def test_lidar_yaml_and_port_override(launch_graph, tmp_path):
    config = tmp_path / 'lidar.yaml'
    config.write_text(yaml.safe_dump({
        'sllidar_node': {'ros__parameters': {
            'serial_port': '/dev/from-config', 'serial_baudrate': 256000,
            'frame_id': 'lidar_link', 'inverted': True,
        }},
    }))
    graph = launch_graph(
        use_rviz='false', lidar_params_file=str(config),
        lidar_dev='/dev/explicit-lidar')
    assert graph.nodes['sllidar_node'].parameters == {
        'serial_port': '/dev/explicit-lidar', 'serial_baudrate': 256000,
        'frame_id': 'lidar_link', 'inverted': True,
    }


@pytest.mark.parametrize('argument', ['lidar_params_file', 'ntrip_params_file'])
def test_missing_stock_profile_config_is_not_ignored(
        launch_graph, tmp_path, argument):
    with pytest.raises(FileNotFoundError, match='parameter file not found'):
        launch_graph(**{argument: str(tmp_path / 'missing.yaml')})


def test_explicit_gnss_config_wins_over_legacy_alias(launch_graph, tmp_path):
    config = tmp_path / 'gnss.yaml'
    config.write_text(yaml.safe_dump({
        'um982_driver': {'ros__parameters': {'baudrate': 38400}},
    }))
    graph = launch_graph(
        use_rviz='false', gnss_params_file=str(config),
        um982_params_file=str(tmp_path / 'unused-legacy.yaml'))
    assert graph.nodes['um982_driver'].parameters['baudrate'] == 38400


def test_ntrip_config_crosses_the_stock_profile_boundary(launch_graph, tmp_path):
    config = tmp_path / 'caster.yaml'
    config.write_text(yaml.safe_dump({
        'ntrip_client': {'ros__parameters': {
            'host': 'caster.example.invalid', 'port': 2101,
        }},
    }))
    graph = launch_graph(use_rviz='false', ntrip_params_file=str(config))
    assert graph.nodes['ntrip_client'].parameters == {
        'host': 'caster.example.invalid', 'port': 2101,
    }
    assert graph.nodes['um982_driver'].parameters['baudrate'] == 115200
