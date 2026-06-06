# outdoor_patrol_loc

M1 localization for the outdoor patrol robot: a single-input
[`robot_localization`](https://github.com/cra-ros-pkg/robot_localization)
EKF that consumes the chassis wheel odometry and produces the REP-105
`odom → base_link` transform.

## Role in the stack

```
ESP32-S3 firmware ──/odom (nav_msgs/Odometry)──▶ ekf_filter_node ──▶ /odometry/filtered
                     frame: odom→base_link                      └──▶ TF: odom → base_link
```

- The firmware publishes the odom **message** at ~100 Hz but does **not**
  broadcast TF. This node is the single owner of `odom → base_link`
  (REP-105 single-writer rule).
- The EKF fuses the body-frame **velocities** (`vx`, `vyaw`) from the lone
  odom source and integrates them, rather than mirroring the source's own
  pose. See [`config/ekf.yaml`](config/ekf.yaml) for the rationale.
- **M2 extends this exact node** with an IMU (`imu0`) — there is no
  throwaway TF broadcaster to rip out.

## Run

```bash
ros2 launch outdoor_patrol_loc localization.launch.py
# override the config or enable sim time:
ros2 launch outdoor_patrol_loc localization.launch.py \
  params_file:=/path/to/ekf.yaml use_sim_time:=false
```

Normally launched as part of the M1 bringup (planned
`outdoor_patrol_bringup/launch/odometry.launch.py`, work stream 3), which
also starts the micro-ROS agent and `robot_state_publisher`. Until that
lands, run this alongside the existing `teleop.launch.py`.

## Firmware contract (prerequisite)

The EKF input is only as good as the odom message. The firmware must:

1. **Stamp `odom.header.stamp`** from synced agent time
   (`rmw_uros_sync_session` + `rmw_uros_epoch_nanos`). The ESP32 has no
   RTC; micro-ROS NTP-syncs to the agent's wall clock. An unsynced/zero
   stamp makes `tf2` and the EKF reject or jitter on the input.
2. **Populate pose/twist covariance** diagonals with non-zero values. The
   EKF weights inputs by covariance; zeros read as "infinitely certain"
   and break fusion the moment the M2 IMU is added.

## Acceptance test (recipe M1)

1. Joystick-drive a taped 2 m × 2 m square; plot `/odometry/filtered` in
   RViz; closure error < 0.3 m.
2. `tf2_echo odom base_link` updates smoothly, no timestamp/jitter
   warnings.
3. `ros2 topic hz /odom` ≥ 20 Hz (firmware runs ~100 Hz).
