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


def render_compose(filename, overrides):
    """Ignore local credentials/env files and return the rendered robot service."""
    environment = {
        key: value for key, value in os.environ.items()
        if key not in DEVICE_ARGS
    }
    environment.update(overrides)
    result = subprocess.run(
        ['docker', 'compose', '--env-file', '/dev/null',
         '-f', str(DEPLOY / filename), 'config', '--format', 'json'],
        env=environment, check=True, capture_output=True, text=True)
    return json.loads(result.stdout)['services']['robot']


@pytest.mark.parametrize('filename', [
    'docker-compose.yaml', 'docker-compose.nav2.yaml',
])
@pytest.mark.parametrize('overrides', [
    {},
    {'GNSS_DEV': '/dev/gnss-new', 'IMU_DEV': '/dev/imu-new'},
    {key: '/dev/test-' + value for key, value in DEVICE_ARGS.items()},
])
def test_device_mapping_matches_driver_port(filename, overrides):
    robot = render_compose(filename, overrides)
    devices = robot['devices']
    arguments = dict(
        value.split(':=', 1) for value in robot['command'] if ':=' in value)
    assert len(devices) == len(DEVICE_ARGS)
    for (variable, argument), device in zip(DEVICE_ARGS.items(), devices):
        assert device['source'] == device['target']
        assert arguments[argument] == device['target']
        if variable in overrides:
            assert device['source'] == overrides[variable]
