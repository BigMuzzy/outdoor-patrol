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
    'SERIAL_DEV': 'serial_dev',
    'GNSS_DEV': 'gnss_dev',
    'IMU_DEV': 'imu_dev',
    'LIDAR_DEV': 'lidar_dev',
}
CONTAINER_PATHS = {
    'SERIAL_DEV': '/dev/op-chassis',
    'GNSS_DEV': '/dev/op-gnss',
    'IMU_DEV': '/dev/op-imu',
    'LIDAR_DEV': '/dev/op-lidar',
}
LEGACY_HOST_PATHS = {
    'SERIAL_DEV': '/dev/ttyACM0',
    'GNSS_DEV': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
    'IMU_DEV': (
        '/dev/serial/by-id/'
        'usb-FTDI_FT230X_Basic_UART_DO01MCPU-if00-port0'),
    'LIDAR_DEV': (
        '/dev/serial/by-id/'
        'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
        'f86253bee863ef11a2a1e2a9c169b110-if00-port0'),
}


def render_compose(
        filename: str, overrides: dict[str, str], *,
        env_file: str | Path = '/dev/null'):
    """Ignore local credentials/env files and return the rendered robot service."""
    environment = {
        key: value for key, value in os.environ.items()
        if key not in DEVICE_ARGS
    }
    environment.update(overrides)
    result = subprocess.run(
        ['docker', 'compose', '--env-file', str(env_file),
         '-f', str(DEPLOY / filename), 'config', '--format', 'json'],
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
        assert arguments[argument] == target
        assert by_target[target]['source'] == overrides.get(
            variable, LEGACY_HOST_PATHS[variable])
    assert robot['environment']['SERIAL_DEV'] == '/dev/op-chassis'


@pytest.mark.parametrize('filename', [
    'docker-compose.yaml', 'docker-compose.nav2.yaml',
])
def test_env_example_enables_host_roles(filename):
    robot = render_compose(filename, {}, env_file=DEPLOY / '.env.example')
    assert {device['source'] for device in robot['devices']} == set(
        CONTAINER_PATHS.values())
    assert all(
        device['source'] == device['target'] for device in robot['devices'])
