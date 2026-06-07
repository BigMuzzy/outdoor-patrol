# imu_driver

ROS 2 driver for **Inertial Labs KERNEL-family IMUs** and other devices that
speak the same Inertial Labs binary wire protocol (`0xAA 0x55` framing, per the
*KERNEL IMU Interface Control Document, rev 1.39*, in
[`docs/`](docs/Kernel_ICD_rev-1.39_Jul_2024.pdf)). The driver streams the
device **Calibrated HR Data** format and publishes `sensor_msgs/Imu` and
`sensor_msgs/Temperature`, with health decoded from the Unit Status Word into
`/diagnostics`.

## Role in the stack

```
IMU (RS-422/USB) ──0xAA55 binary──▶ imu_driver ──▶ ~/data        (sensor_msgs/Imu)
                                              └──▶ ~/temperature (sensor_msgs/Temperature)
                                              └──▶ /diagnostics  (USW, rate, BIT, dev info)
```

The driver is the `imu0` source for the `outdoor_patrol_loc` EKF (M2). It only
**reads** the device and sends the runtime start/stop command — it never writes
flash. Configure data rate, baud, alignment angles, and output variant with the
Inertial Labs GUI beforehand.

## Sensor frame (important)

Data is published **raw, in the IMU's own sensor axes**:

| Axis | Direction          |
|------|--------------------|
| X    | lateral / right    |
| Y    | longitudinal / fwd |
| Z    | normal / up        |

This is **not** the REP-103 `base_link` convention (X-forward, Y-left, Z-up).
Define `frame_id` (default `imu_link`) in the robot URDF and provide a static
transform `base_link → imu_link` describing the physical mounting. The
orientation quaternion follows the device convention (ENU world frame,
clockwise-positive heading) per Appendix D of the ICD.

> The Calibrated HR heading is **relative** (these IMUs have no magnetometer),
> so absolute yaw drifts. `orientation_covariance` defaults to a large yaw
> variance; for 2D fusion prefer the gyro `angular_velocity.z` (`vyaw`).

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/serial/by-id/usb-FTDI_FT230X_…` | Serial device path (stable `by-id`; matches the `--device=` passthrough). |
| `baudrate` | `2000000` | Serial baud. **Must equal the device's COM baud** (see below). |
| `frame_id` | `imu_link` | Frame for published messages. |
| `output_format` | `calib_hr` | Device output format (only `calib_hr` implemented). |
| `gravity` | `9.8106` | g → m/s² conversion (per ICD). |
| `publish_orientation` | `true` | Publish the orientation quaternion. |
| `publish_temperature` | `true` | Publish `~/temperature`. |
| `query_device_info` | `true` | Send GetDevInfo/GetBIT on activation. |
| `publish_every_n` | `20` | Publish 1 of every N parsed samples (decimation). `1` = every sample; `20` throttles a 2000 Hz device to ~100 Hz. The device is still read at full rate. |
| `expected_rate_hz` | `2000.0` | Expected **device** rate for the diagnostics lower-bound check (checks `sample_rate_hz`, not the decimated `publish_rate_hz`). |
| `orientation_covariance` | diag(0.0025, 0.0025, 100) | 9-element row-major. |
| `angular_velocity_covariance` | diag(4e-4) | 9-element row-major. |
| `linear_acceleration_covariance` | diag(0.04) | 9-element row-major. |

## Run

This is a **lifecycle** node; `ros2 run` does not auto-activate it.

```bash
ros2 launch imu_driver imu_driver.launch.py \
  params_file:=src/imu_driver/config/imu_driver.yaml

# override just the baud without editing the YAML:
ros2 launch imu_driver imu_driver.launch.py baud:=2000000

# drive the lifecycle
ros2 lifecycle set /imu_driver configure
ros2 lifecycle set /imu_driver activate

# observe (publish_rate_hz is post-decimation; sample_rate_hz is the device)
ros2 topic hz /imu_driver/data
ros2 topic echo /diagnostics --once
```

### Rate / decimation

The device streams Calibrated HR much faster than a ground-robot EKF needs
(this unit auto-streams 2000 Hz). Rather than reconfiguring the device, the
driver **reads every frame at full rate** (so USW faults are caught promptly)
but **publishes only 1 of every `publish_every_n`** on `~/data` / `~/temperature`.
With `publish_every_n: 20` a 2000 Hz device yields ~100 Hz published. Diagnostics
reports both `sample_rate_hz` (device) and `publish_rate_hz` (after decimation).

## Device data rate & baud — read before changing the rate in the GUI

The driver **does not configure the device** (no `SaveFlash`). The output data
rate and the COM-port baud live in the IMU's flash and are set with the
**Inertial Labs GUI**; with `Auto_Start` the unit streams its chosen format on
power-up. The driver only matches what the device already does.

**Three settings must agree. The driver owns only the last one:**

| # | Setting | Where | Notes |
|---|---------|-------|-------|
| 1 | Output **data rate** (Hz) | device flash (GUI) | Must be a factor of 2000 Hz. |
| 2 | Device **COM baud** | device flash (GUI) | Must be fast enough for rate 1 (ICD §6.5), or the device silently drops the rate. |
| 3 | Driver `baudrate` | this package's config | **Must equal setting 2.** The driver never changes the device baud. |
| 4 | Driver `expected_rate_hz` | this package's config | Should equal setting 1. Cosmetic — only the diagnostics rate-check WARN threshold. |

Max CalibHR rate is capped by baud, `max ≈ baud / (11 × 60 bytes)` rounded down
to a factor of 2000 (ICD Table 6.27): **125 Hz @ 115200**, 500 Hz @ 460800,
**2000 Hz @ 2 000 000**.

> **As tested 2026-06-07** (KERNEL-201, SN `K58F8162`): the unit ships
> auto-streaming Calibrated HR at **2000 Hz**, which needs **2 Mbaud** — hence
> the config defaults to `baudrate: 2000000`.

### If you lower the rate in the GUI later

1. In the GUI set the new **data rate** and (optionally) a lower **COM baud**.
2. Set the driver `baudrate` to **exactly** the device's COM baud.
3. Set `expected_rate_hz` to the new data rate.
4. Re-`configure`/`activate` and confirm `/diagnostics`.

### Symptom of a baud mismatch (so you don't guess)

If the driver `baudrate` ≠ the device COM baud, the bytes are garbled and
**occasionally false-sync into "valid"-looking frames with a random
`data_id`**. In `/diagnostics` you'll see **`unknown_frames` climbing**
(≈ a few per second) together with non-zero **`bytes_discarded`** and possibly
**`checksum_errors`**, while `sample_rate_hz` is wrong or unstable. At a matched
baud all three counters sit at ~0. So if the IMU data looks wrong, **check those
counters first** — a `baudrate` vs device-COM-baud mismatch is the usual cause,
not the parser.

## Protocol summary

- Frame: `AA 55 | msg_type | data_id | length(2, LE) | payload | checksum(2, LE)`
  where `length = payload_len + 6` and the checksum is the 16-bit arithmetic
  sum of every byte from `msg_type` through the last payload byte.
- Start streaming: send the 1-byte command (e.g. `0x81` Calibrated HR); the
  device echoes the format on `data_id` in each output frame.
- Calibrated HR payload (52 bytes): heading/pitch/roll (deg·1000, int32),
  gyro XYZ (deg/s·1e5, int32), accel XYZ (g·1e6, int32), mag (0), counter, USW,
  temperature (°C·10). All multi-byte fields are little-endian.

## Build & test

```bash
colcon build --merge-install --packages-select imu_driver
colcon test --merge-install --packages-select imu_driver
colcon test-result --verbose
```

## Roadmap

- User Defined Data (0x95/0x96): quaternion + HR sensors + USW-extended + 1PPS
  time in one configurable packet.
- Low-res Orientation (0x33) with KG/KA range read (ReadRAM 0x8F).
- Optional SaveFlash configuration (data rate, alignment angles, baud, LPF).
- 1PPS / IMU-time stamping (GAm formats) for tight time sync.
- Wire `imu0` into `outdoor_patrol_loc/config/ekf.yaml` (M2).
