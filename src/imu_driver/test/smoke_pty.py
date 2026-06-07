#!/usr/bin/env python3
# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Smoke-test the imu_driver node against a synthetic Calibrated HR stream."""
# Feeds a fabricated AA55 CalibHR byte stream into a pseudo-terminal and checks
# that the node publishes sensor_msgs/Imu on ~/data.
import os
import pty
import struct
import subprocess
import sys
import threading
import time


def build_calib_hr_frame(heading_mdeg, pitch_mdeg, roll_mdeg,
                         gyro_e5, acc_e6, counter, usw, temper_d):
    """Build one AA55 'Calibrated HR Data' (0x81) frame."""
    payload = bytearray()
    payload += struct.pack('<iii', heading_mdeg, pitch_mdeg, roll_mdeg)
    payload += struct.pack('<iii', *gyro_e5)       # gyro XYZ, deg/s * 1e5
    payload += struct.pack('<iii', *acc_e6)        # accel XYZ, g * 1e6
    payload += struct.pack('<hhh', 0, 0, 0)        # mag (not measured)
    payload += struct.pack('<H', counter)          # counter
    payload += struct.pack('<H', 0)                # reserved
    payload += struct.pack('<H', usw)              # USW
    payload += struct.pack('<H', 0)                # reserved
    payload += struct.pack('<h', temper_d)         # temperature, C * 10
    assert len(payload) == 52, len(payload)

    length = len(payload) + 6
    frame = bytearray([0xAA, 0x55, 0x01, 0x81])
    frame += struct.pack('<H', length)
    frame += payload
    checksum = sum(frame[2:]) & 0xFFFF
    frame += struct.pack('<H', checksum)
    return bytes(frame)


def main():
    """Run the PTY smoke test and return a process exit code."""
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print(f'PTY slave={slave_name}', flush=True)

    # Drain commands the driver writes (Stop/GetDevInfo/GetBIT/CalibHR).
    def drain():
        while True:
            try:
                os.read(master, 4096)
            except OSError:
                return

    threading.Thread(target=drain, daemon=True).start()

    def feed():
        time.sleep(3.0)
        # heading 30 deg, level, gyroZ 0.5 deg/s, accZ 1 g, 25.0 C.
        frame = build_calib_hr_frame(
            heading_mdeg=30000, pitch_mdeg=0, roll_mdeg=0,
            gyro_e5=(0, 0, 50000), acc_e6=(0, 0, 1000000),
            counter=1, usw=0, temper_d=250)
        for _ in range(400):
            try:
                os.write(master, frame)
            except OSError:
                return
            time.sleep(0.02)  # ~50 Hz

    threading.Thread(target=feed, daemon=True).start()

    driver = subprocess.Popen([
        'ros2', 'run', 'imu_driver', 'imu_driver_node',
        '--ros-args',
        '-p', f'port:={slave_name}',
        '-p', 'baudrate:=115200',
        '-p', 'query_device_info:=false',
    ])

    time.sleep(1.5)
    subprocess.run(['ros2', 'lifecycle', 'set', '/imu_driver', 'configure'],
                   check=False)
    time.sleep(0.5)
    subprocess.run(['ros2', 'lifecycle', 'set', '/imu_driver', 'activate'],
                   check=False)

    echoer = subprocess.run(
        ['ros2', 'topic', 'echo', '--once', '--qos-profile', 'sensor_data',
         '/imu_driver/data'],
        capture_output=True, text=True, timeout=15)
    print('--- ros2 topic echo /imu_driver/data ---')
    print(echoer.stdout)
    if echoer.returncode != 0:
        print('STDERR:', echoer.stderr, file=sys.stderr)

    driver.terminate()
    try:
        driver.wait(timeout=5)
    except subprocess.TimeoutExpired:
        driver.kill()

    # accZ = 1.0 g -> 9.8106 m/s^2.
    ok = '9.8106' in echoer.stdout
    print('OK' if ok else 'MISSING_IMU')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
