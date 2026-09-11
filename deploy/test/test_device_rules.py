# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Exercise the host installer with fake udev and isolated filesystem roots."""

from dataclasses import dataclass
import os
from pathlib import Path
import subprocess

import pytest


REPO = Path(__file__).resolve().parents[2]
TEMPLATE = REPO / 'deploy/udev/99-outdoor-patrol.rules'
OLD_CHASSIS_RULE = (
    'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", '
    'ATTRS{serial}=="1C:DB:D4:7A:29:9C", SYMLINK+="esp32-chassis"\n'
)


@dataclass
class FakeHost:
    """Only subprocesses pointed at these temporary roots may be run."""

    rules: Path
    devices: Path
    source: Path
    log: Path
    environment: dict

    def run(self, *arguments, wrapper=False, **environment):
        """Invoke the installer with udev/sudo mocked, never real host paths."""
        script = (
            '.devcontainer/install-udev-rules.sh' if wrapper
            else 'scripts/install-device-rules.sh')
        return subprocess.run(
            ['/bin/bash', str(REPO / script), *map(str, arguments)],
            env={**self.environment, **environment},
            text=True, capture_output=True, check=False)

    def calls(self):
        """Return calls received by the fake udev tool."""
        return self.log.read_text().splitlines() if self.log.exists() else []


@pytest.fixture
def host(tmp_path):
    """Make an entirely fake installation target and udev command."""
    rules = tmp_path / 'rules'
    devices = tmp_path / 'dev'
    bin_dir = tmp_path / 'bin'
    for directory in (rules, devices, bin_dir):
        directory.mkdir()
    for name, text in {
        'udevadm': (
            '#!/bin/sh\nprintf "%s\\n" "$*" >> "$UDEV_LOG"\n'
            'exit "${UDEV_EXIT:-0}"\n'),
        'sudo': '#!/bin/sh\nexec "$@"\n',
        'stat': (
            '#!/bin/sh\nprintf "%s\\n" "${TEST_DEVICE_GROUP:-dialout}"\n'),
    }.items():
        tool = bin_dir / name
        tool.write_text(text)
        tool.chmod(0o755)
    source = tmp_path / 'robot.rules'
    source.write_text(TEMPLATE.read_text().replace(
        '@GNSS_ID_PATH@', 'pci-test-usb-0:4:1.0'))
    log = tmp_path / 'udev.log'
    return FakeHost(rules, devices, source, log, {
        **os.environ,
        'PATH': str(bin_dir) + ':' + os.environ['PATH'],
        'OUTDOOR_PATROL_RULES_DIR': str(rules),
        'OUTDOOR_PATROL_DEV_DIR': str(devices),
        'UDEV_LOG': str(log),
        'UDEV_EXIT': '0',
        'TEST_DEVICE_GROUP': 'dialout',
        'TMPDIR': str(tmp_path),
    })


def test_unconfigured_gnss_is_refused(host):
    result = host.run()
    assert result.returncode != 0
    assert 'GNSS identity is unset' in result.stderr
    assert list(host.rules.iterdir()) == []
    assert host.calls() == []


def test_chassis_wrapper_is_optional_and_idempotent(host):
    first = host.run(wrapper=True)
    assert first.returncode == 0, first.stderr
    installed = (host.rules / '99-esp32-chassis.rules').read_text()
    assert 'op-chassis esp32-chassis' in installed
    assert 'op-gnss' not in installed
    assert 'op-imu' not in installed
    calls = host.calls()
    assert calls == [
        'control --reload-rules',
        'trigger --subsystem-match=tty',
    ]
    second = host.run(wrapper=True, UDEV_EXIT='9')
    assert second.returncode == 0, second.stderr
    assert 'already up to date' in second.stdout
    assert host.calls() == calls


def test_full_install_migrates_only_known_legacy_rules(host):
    legacy = host.rules / '99-esp32-chassis.rules'
    legacy.write_text('# Original project rule\n' + OLD_CHASSIS_RULE)
    result = host.run('--rules-file', host.source)
    assert result.returncode == 0, result.stderr
    target = host.rules / '99-outdoor-patrol.rules'
    assert target.read_text() == host.source.read_text()
    assert target.stat().st_mode & 0o777 == 0o644
    assert not legacy.exists()
    assert len(host.calls()) == 3
    second = host.run('--rules-file', host.source)
    assert second.returncode == 0, second.stderr
    assert len(host.calls()) == 6


def test_full_host_configuration_survives_devcontainer_start(host):
    target = host.rules / '99-outdoor-patrol.rules'
    original = host.source.read_text().replace('DO01MCPU', 'new-unit-serial')
    target.write_text(original)
    result = host.run(wrapper=True)
    assert result.returncode == 0, result.stderr
    assert 'leaving them unchanged' in result.stdout
    assert target.read_text() == original
    assert not (host.rules / '99-esp32-chassis.rules').exists()
    assert host.calls() == []


@pytest.mark.parametrize('wrapper', [True, False])
def test_custom_legacy_rule_is_not_overwritten(host, wrapper):
    legacy = host.rules / '99-esp32-chassis.rules'
    original = OLD_CHASSIS_RULE.replace('1C:DB:D4:7A:29:9C', 'custom-unit')
    legacy.write_text(original)
    arguments = [] if wrapper else ['--rules-file', host.source]
    result = host.run(*arguments, wrapper=wrapper)
    if wrapper:
        assert result.returncode == 0
        assert 'customized chassis rule left unchanged' in result.stderr
    else:
        assert result.returncode != 0
        assert 'Legacy rules were customized' in result.stderr
    assert legacy.read_text() == original
    assert not (host.rules / '99-outdoor-patrol.rules').exists()
    assert host.calls() == []


def test_missing_role_is_refused(host):
    host.source.write_text('\n'.join(
        line for line in host.source.read_text().splitlines()
        if 'op-imu' not in line))
    result = host.run('--rules-file', host.source)
    assert result.returncode != 0
    assert 'Missing op-imu alias' in result.stderr
    assert list(host.rules.iterdir()) == []
    assert host.calls() == []


def test_verification_fails_loudly_on_missing_hardware(host):
    result = host.run('--rules-file', host.source, '--verify')
    assert result.returncode != 0
    assert 'Missing character device' in result.stderr


@pytest.mark.parametrize('duplicate, group, error', [
    (False, 'dialout', None),
    (True, 'dialout', 'shares'),
    (False, 'plugdev', 'expected dialout'),
])
def test_verify_alias_targets_and_permissions(host, duplicate, group, error):
    targets = ['/dev/null', '/dev/zero', '/dev/full', '/dev/random']
    if duplicate:
        targets[2] = targets[1]
    for role, target in zip(('chassis', 'gnss', 'imu', 'lidar'), targets):
        (host.devices / ('op-' + role)).symlink_to(target)
    result = host.run(
        '--rules-file', host.source, '--verify', TEST_DEVICE_GROUP=group)
    if error is None:
        assert result.returncode == 0, result.stderr
        assert result.stdout.count('verified ') == 4
    else:
        assert result.returncode != 0
        assert error in result.stderr


def test_reload_failure_is_not_reported_as_success(host):
    result = host.run('--rules-file', host.source, UDEV_EXIT='9')
    assert result.returncode == 9
    assert 'installed and reloaded' not in result.stdout
    assert host.calls() == ['control --reload-rules']
    retry = host.run('--rules-file', host.source)
    assert retry.returncode == 0, retry.stderr
    assert host.calls() == [
        'control --reload-rules',
        'control --reload-rules',
        'trigger --subsystem-match=tty',
        'settle --timeout=10',
    ]


def test_explicit_chassis_verification_retries_activation(host):
    (host.devices / 'op-chassis').symlink_to('/dev/null')
    failed = host.run('--chassis-only', UDEV_EXIT='9')
    assert failed.returncode == 9
    retry = host.run('--chassis-only', '--verify')
    assert retry.returncode == 0, retry.stderr
    assert host.calls() == [
        'control --reload-rules',
        'control --reload-rules',
        'trigger --subsystem-match=tty',
        'settle --timeout=10',
    ]


def test_devcontainer_without_udev_still_starts(host, tmp_path):
    no_tools = tmp_path / 'no-tools'
    no_tools.mkdir()
    result = host.run(wrapper=True, PATH=str(no_tools))
    assert result.returncode == 0
    assert 'udevadm not found; skipping' in result.stdout
    assert host.calls() == []
