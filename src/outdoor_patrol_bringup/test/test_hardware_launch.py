# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Characterize the real robot's launch-time hardware and safety contracts."""

import pytest
import yaml


DEFAULT_NODES = {
    'micro_ros_agent', 'robot_state_publisher',
    'ntrip_client', 'um982_driver',
    'ekf_filter_node', 'ekf_global', 'heading_to_imu',
    'confidence_gate', 'navsat_transform',
    'imu_driver', 'sllidar_node', 'scan_box_filter', 'scan_safety',
}


def test_default_hardware_graph(launch_graph):
    graph = launch_graph(use_rviz='false')
    assert set(graph.nodes) == DEFAULT_NODES
    assert graph.configure == {'um982_driver', 'imu_driver'}
    assert graph.activate_on_inactive == graph.configure
    assert graph.nodes['imu_driver'].delay == 12.0
    for name in ('sllidar_node', 'scan_box_filter', 'scan_safety'):
        assert graph.nodes[name].delay == 8.0
    assert graph.nodes['micro_ros_agent'].arguments[:5] == (
        'serial', '--dev', '/dev/ttyACM0', '-b', '115200')


def test_driver_parameters_and_imu_config_regression(launch_graph):
    nodes = launch_graph(use_rviz='false').nodes
    assert nodes['um982_driver'].parameters['port'] == (
        '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    assert nodes['um982_driver'].parameters['baudrate'] == 115200
    imu = nodes['imu_driver']
    assert [path.name for path in imu.parameter_files] == ['imu_driver.yaml']
    assert imu.parameters['port'] == (
        '/dev/serial/by-id/'
        'usb-FTDI_FT230X_Basic_UART_DO01MCPU-if00-port0')
    assert imu.parameters['baudrate'] == 2000000
    assert imu.parameters['publish_every_n'] == 20
    assert imu.parameters['frame_id'] == 'imu_link'
    assert nodes['sllidar_node'].parameters == {
        'channel_type': 'serial',
        'serial_port': (
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
            'f86253bee863ef11a2a1e2a9c169b110-if00-port0'),
        'serial_baudrate': 460800,
        'frame_id': 'lidar_link',
        'inverted': False,
        'angle_compensate': True,
        'scan_mode': 'Standard',
    }


def test_corrections_and_scan_safety_wiring(launch_graph):
    nodes = launch_graph(use_rviz='false').nodes
    assert nodes['um982_driver'].remappings == (('rtcm/in', '/rtcm'),)
    assert nodes['ntrip_client'].remappings == (
        ('rtcm/out', '/rtcm'),
        ('nmea_sentence', '/um982_driver/nmea_sentence'),
    )
    assert nodes['sllidar_node'].remappings == (('scan', '/scan_raw'),)
    assert nodes['scan_box_filter'].remappings == (
        ('scan', '/scan_raw'), ('scan_filtered', '/scan'),
    )
    assert nodes['scan_safety'].remappings == (
        ('scan', '/scan'),
        ('cmd_vel_in', '/cmd_vel_raw'),
        ('cmd_vel_out', '/cmd_vel'),
    )
    assert nodes['scan_safety'].parameters == {
        'sector_half_angle_deg': 30.0,
        'forward_offset_deg': 180.0,
        'stop_distance_m': 0.5,
        'min_range_m': 0.06,
        'scan_timeout_s': 0.5,
        'cmd_timeout_s': 0.5,
        'control_period_s': 0.05,
    }
    assert nodes['scan_box_filter'].parameters == {
        'filter1': {
            'name': 'body_box',
            'type': 'laser_filters/LaserScanBoxFilter',
            'params': {
                'box_frame': 'base_link',
                'min_x': -0.15, 'max_x': 0.59,
                'min_y': -0.27, 'max_y': 0.27,
                'min_z': -1.0, 'max_z': 1.0,
                'invert': False,
            },
        },
    }
    assert nodes['ekf_filter_node'].parameters['odom0'] == '/odom'
    assert nodes['ekf_global'].parameters['imu1'] == '/imu_driver/data'
    assert nodes['ekf_global'].parameters['imu0'] == '/gnss/heading'
    assert nodes['ekf_global'].parameters['odom1'] == '/odometry/gps'


@pytest.mark.parametrize('overrides, omitted', [
    ({'use_imu': 'false'}, {'imu_driver'}),
    ({'use_lidar': 'false'},
     {'sllidar_node', 'scan_box_filter', 'scan_safety'}),
    ({'use_imu': 'false', 'use_lidar': 'false'},
     {'imu_driver', 'sllidar_node', 'scan_box_filter', 'scan_safety'}),
])
def test_optional_groups(launch_graph, overrides, omitted):
    graph = launch_graph(use_rviz='false', **overrides)
    assert set(graph.nodes) == DEFAULT_NODES - omitted


def test_legacy_auto_activate_currently_disables_both_drivers(launch_graph):
    graph = launch_graph(use_rviz='false', auto_activate='false')
    assert set(graph.nodes) == DEFAULT_NODES
    assert graph.configure == set()
    assert graph.activate_on_inactive == set()


def test_generic_ekf_params_never_replace_imu_params(launch_graph, tmp_path):
    params = tmp_path / 'local_ekf.yaml'
    params.write_text(yaml.safe_dump({
        'ekf_filter_node': {'ros__parameters': {'frequency': 17.0}},
    }))
    graph = launch_graph(use_rviz='false', params_file=str(params))
    assert set(graph.nodes) == DEFAULT_NODES
    assert graph.nodes['ekf_filter_node'].parameters['frequency'] == 17.0
    assert graph.nodes['imu_driver'].parameters['baudrate'] == 2000000


@pytest.mark.parametrize('package, launch_file, expected, active', [
    ('um982_driver', 'um982.launch.py', {'um982_driver'}, set()),
    ('um982_driver', 'gnss_rtk.launch.py',
     {'um982_driver', 'ntrip_client'}, {'um982_driver'}),
    ('imu_driver', 'imu_driver.launch.py', {'imu_driver'}, {'imu_driver'}),
])
def test_standalone_driver_lifecycle(
        launch_graph, package, launch_file, expected, active):
    graph = launch_graph(
        package, launch_file, use_rviz='false', use_static_tf='false')
    assert set(graph.nodes) == expected
    assert graph.configure == active
    assert graph.activate_on_inactive == active
