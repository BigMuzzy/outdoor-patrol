# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Render Compose without opening devices or starting containers."""

import json
import os
from pathlib import Path
import subprocess

import pytest


DEPLOY = Path(__file__).resolve().parents[1]
DEVICE_ARGS = {
    'CHASSIS_DEV': 'chassis_dev',
    'GNSS_DEV': 'gnss_dev',
    'IMU_DEV': 'imu_dev',
    'LIDAR_DEV': 'lidar_dev',
}
CONTAINER_PATHS = {
    'CHASSIS_DEV': '/dev/op-chassis',
    'GNSS_DEV': '/dev/op-gnss',
    'IMU_DEV': '/dev/op-imu',
    'LIDAR_DEV': '/dev/op-lidar',
}
LEGACY_HOST_PATHS = {
    'CHASSIS_DEV': '/dev/ttyACM0',
    'GNSS_DEV': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
    'IMU_DEV': (
        '/dev/serial/by-id/'
        'usb-FTDI_FT230X_Basic_UART_DO01MCPU-if00-port0'),
    'LIDAR_DEV': (
        '/dev/serial/by-id/'
        'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
        'f86253bee863ef11a2a1e2a9c169b110-if00-port0'),
}
PROFILE_VARIABLES = {
    role + '_' + suffix
    for role in ('GNSS', 'IMU', 'LIDAR')
    for suffix in ('LAUNCH_FILE', 'PARAMS', 'PORT')
}


def render_compose(
        filename: str, overrides: dict[str, str], *,
        env_file: str | Path = '/dev/null',
        override_files: tuple[str, ...] = ()):
    """Ignore local credentials/env files and return the rendered robot service."""
    environment = {
        key: value for key, value in os.environ.items()
        if key not in (
            set(DEVICE_ARGS) | PROFILE_VARIABLES |
            {'NTRIP_PARAMS', 'SERIAL_DEV', 'SERIAL_BAUD', 'DATA_DIR'})
    }
    environment.update(overrides)
    files = ['-f', str(DEPLOY / filename)]
    for override in override_files:
        files.extend(['-f', str(DEPLOY / override)])
    binary = os.environ.get('OUTDOOR_PATROL_COMPOSE_BIN')
    command = [binary] if binary else ['docker', 'compose']
    result = subprocess.run(
        [*command, '--env-file', str(env_file),
         *files, 'config', '--format', 'json'],
        env=environment, check=True, capture_output=True, text=True)
    return json.loads(result.stdout)['services']['robot']


@pytest.mark.parametrize('filename', [
    'docker-compose.yaml', 'docker-compose.nav2.yaml',
])
@pytest.mark.parametrize('overrides', [
    {},
    CONTAINER_PATHS,
    {'GNSS_DEV': '/dev/gnss-new', 'IMU_DEV': '/dev/imu-new'},
    {key: '/dev/test-' + value for key, value in DEVICE_ARGS.items()},
])
def test_device_mapping_matches_driver_port(filename, overrides):
    robot = render_compose(filename, overrides)
    devices = robot['devices']
    arguments = dict(
        value.split(':=', 1) for value in robot['command'] if ':=' in value)
    assert len(devices) == len(DEVICE_ARGS)
    by_target = {device['target']: device for device in devices}
    assert set(by_target) == set(CONTAINER_PATHS.values())
    for variable, argument in DEVICE_ARGS.items():
        target = CONTAINER_PATHS[variable]
        if variable == 'CHASSIS_DEV':
            assert arguments[argument] == target
        else:
            assert argument not in arguments
            assert robot['environment'][variable.replace('_DEV', '_PORT')] == (
                target)
        assert by_target[target]['source'] == overrides.get(
            variable, LEGACY_HOST_PATHS[variable])
    assert robot['environment']['CHASSIS_DEV'] == '/dev/op-chassis'
    assert robot['environment']['SERIAL_DEV'] == '/dev/op-chassis'
    assert 'serial_dev' not in arguments
    assert not any(value.endswith(':=') for value in robot['command'])


@pytest.mark.parametrize('filename', [
    'docker-compose.yaml', 'docker-compose.nav2.yaml',
])
def test_env_example_enables_host_roles(filename):
    robot = render_compose(filename, {}, env_file=DEPLOY / '.env.example')
    assert {device['source'] for device in robot['devices']} == set(
        CONTAINER_PATHS.values())
    assert all(
        device['source'] == device['target'] for device in robot['devices'])


@pytest.mark.parametrize('filename', [
    'docker-compose.yaml', 'docker-compose.nav2.yaml',
])
def test_mounted_profile_and_params_overrides(filename):
    overrides = {
        role + '_' + suffix: '/data/' + role.lower() + extension
        for role in ('GNSS', 'IMU', 'LIDAR')
        for suffix, extension in (
            ('LAUNCH_FILE', '.launch.py'), ('PARAMS', '.yaml'))
    }
    robot = render_compose(filename, overrides)
    arguments = dict(
        value.split(':=', 1) for value in robot['command'] if ':=' in value)
    for role in ('gnss', 'imu', 'lidar'):
        assert arguments[role + '_launch_file'] == (
            '/data/' + role + '.launch.py')
        assert role + '_params_file' not in arguments
        assert robot['environment'][role.upper() + '_PARAMS'] == (
            '/data/' + role + '.yaml')
    assert any(volume['target'] == '/data' for volume in robot['volumes'])


@pytest.mark.parametrize('filename', [
    'docker-compose.yaml', 'docker-compose.nav2.yaml',
])
def test_network_lidar_removes_only_its_usb_binding(filename):
    robot = render_compose(filename, {
        **CONTAINER_PATHS,
        'LIDAR_PORT': '',
        'LIDAR_LAUNCH_FILE': '/data/network_lidar.launch.py',
    }, override_files=('docker-compose.network-lidar.yaml',))
    devices = {device['target']: device['source'] for device in robot['devices']}
    expected = set(CONTAINER_PATHS.values()) - {'/dev/op-lidar'}
    assert devices == {path: path for path in expected}
    arguments = dict(
        value.split(':=', 1) for value in robot['command'] if ':=' in value)
    assert 'lidar_dev' not in arguments
    assert robot['environment']['LIDAR_PORT'] == ''
    assert arguments['lidar_launch_file'] == '/data/network_lidar.launch.py'
    assert arguments.get('use_lidar', 'true') == 'true'
    assert arguments['chassis_dev'] == '/dev/op-chassis'
    assert robot['environment']['GNSS_PORT'] == '/dev/op-gnss'
    assert robot['environment']['IMU_PORT'] == '/dev/op-imu'


@pytest.mark.parametrize('filename', [
    'docker-compose.yaml', 'docker-compose.nav2.yaml',
])
@pytest.mark.parametrize('overlay', [
    (), ('docker-compose.network-lidar.yaml',),
])
@pytest.mark.parametrize('overrides, source', [
    ({'SERIAL_DEV': '/dev/chassis-legacy'}, '/dev/chassis-legacy'),
    ({'SERIAL_DEV': '/dev/chassis-legacy', 'CHASSIS_DEV': '/dev/chassis-new'},
     '/dev/chassis-new'),
    ({'SERIAL_DEV': '/dev/chassis-legacy', 'CHASSIS_DEV': ''},
     '/dev/chassis-legacy'),
    ({'SERIAL_DEV': '', 'CHASSIS_DEV': ''}, '/dev/ttyACM0'),
])
def test_chassis_host_alias_and_precedence(filename, overlay, overrides, source):
    robot = render_compose(filename, overrides, override_files=overlay)
    devices = {device['target']: device['source'] for device in robot['devices']}
    assert devices['/dev/op-chassis'] == source
    arguments = dict(
        value.split(':=', 1) for value in robot['command'] if ':=' in value)
    assert arguments['chassis_dev'] == '/dev/op-chassis'
    assert 'serial_dev' not in arguments
    assert robot['environment']['CHASSIS_DEV'] == '/dev/op-chassis'
    assert robot['environment']['SERIAL_DEV'] == '/dev/op-chassis'
