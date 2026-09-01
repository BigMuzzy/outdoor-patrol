# outdoor_patrol_sim

Gazebo (Harmonic / `gz-sim` 8) simulation of the outdoor patrol robot. It
stands in for the entire physical stack — ESP32-S3 chassis controller, UM982
RTK GNSS, UMKA IMU-E73, RPLIDAR C1 — so you can develop and regression-test
the localization and safety stack with no hardware and no field trip.

## Design rule: same topics, same frames, no remaps

The simulation publishes on the **same ROS topic names the real drivers use**.
`outdoor_patrol_loc` runs against it completely unmodified — that is the whole
point, and it is the constraint to preserve when extending this package.

| Topic | Type | Real source | Sim source |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | ESP32-S3 firmware (sink) | `gz-sim-diff-drive-system` |
| `/odom` | `nav_msgs/Odometry` | ESP32-S3 firmware | DiffDrive → `odom_sim` |
| `/joint_states` | `sensor_msgs/JointState` | — (wheels are fixed on the robot) | `gz-sim-joint-state-publisher-system` |
| `/scan` | `sensor_msgs/LaserScan` | `sllidar_ros2` (RPLIDAR C1) | `gpu_lidar` sensor |
| `/imu_driver/data` | `sensor_msgs/Imu` | `imu_driver` (UMKA IMU-E73) | `imu` sensor |
| `/um982_driver/fix` | `sensor_msgs/NavSatFix` | `um982_driver` + NTRIP | `navsat` sensor → `gnss_sim` |
| `/gnss/heading` | `sensor_msgs/Imu` | `heading_to_imu` (dual antenna) | `gnss_sim` (from ground truth) |
| `/odom_truth` | `nav_msgs/Odometry` | **does not exist** | `gz-sim-odometry-publisher-system` |

`/odom_truth` is Gazebo's ground truth. Use it to score the EKF; **never fuse
it**.

## Run it

```bash
./build.sh && source install/setup.bash

# Terminal 1: headless sim + dual-EKF + GNSS (map -> odom -> base_link)
ros2 launch outdoor_patrol_sim sim.launch.py

# Terminal 2: keyboard teleop (needs a real TTY, not auto-launched)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3 (optional): RViz, ground truth overlaid on the EKF estimate
ros2 launch outdoor_patrol_sim sim.launch.py use_rviz:=true
```

Useful arguments:

| Argument | Default | Effect |
| --- | --- | --- |
| `gui` | `false` | `true` opens the Gazebo GUI (needs a display). |
| `use_rviz` | `false` | RViz with [`config/sim.rviz`](config/sim.rviz), fixed frame `map`. |
| `localization` | `true` | `false` skips the dual-EKF; no `map`/`odom` TF. |
| `gnss` | `true` | `false` skips `gnss_sim`; no fix, no heading. |
| `safety` | `false` | `true` splices the M3 forward brake between `/cmd_vel_raw` and `/cmd_vel`. |
| `world` | `worlds/patrol_yard.sdf` | Any SDF world. |
| `x` / `y` / `yaw` | `0` | Spawn pose. |
| `software_rendering` | `false` | Force llvmpipe (only for hosts with no `/dev/dri`). |

Quick check that it is alive:

```bash
ros2 topic echo /odom_truth --once --field pose.pose.position
ros2 topic hz /scan          # ~10 Hz
ros2 run tf2_ros tf2_echo map base_link
```

## Where the geometry comes from

Nowhere in this package. The robot description is
[`outdoor_patrol_bringup`'s xacro](../outdoor_patrol_bringup/urdf/outdoor_patrol.urdf.xacro)
included with `sim:=true`, which is what adds `<collision>` / `<inertial>`
blocks and turns the wheel joints `continuous`. Dimensions, mount poses and
speed clamps all still live in
[`config/chassis.yaml`](../outdoor_patrol_bringup/config/chassis.yaml).
[`urdf/outdoor_patrol_sim.urdf.xacro`](urdf/outdoor_patrol_sim.urdf.xacro)
adds only the things that exist solely in simulation: a frictionless front
caster, wheel friction, the gz system plugins, and the three sensors.

## The two shim nodes, and why they are not optional

Both exist because a gz message is missing a field that the localization stack
treats as load-bearing.

**`odom_sim`** — the gz DiffDrive plugin publishes an all-zero covariance
matrix, which `robot_localization` reads as "infinitely trustworthy". The EKF
would ride perfect simulated wheel odometry and ignore GNSS entirely. This
node stamps the same fixed diagonal the firmware sends (see
[`config/odom_covariance.yaml`](config/odom_covariance.yaml), which must stay
in lock-step with the "Static odom covariance diagonals" block in
`src/esp32-s3-uros-controller/firmware/main/uros_task.c`).

**`gnss_sim`** — fills three gaps:

1. `gz.msgs.NavSat` carries no covariance, so `confidence_gate` has nothing to
   grade the fix on.
2. gz-sim's navsat `<position_sensing>` noise is applied to latitude and
   longitude **in degrees**, so a plausible-looking `0.02` throws the fix
   kilometres off course. The sensor is therefore configured noise-free and
   the noise is added here in metres.
3. The gz navsat sensor is single-antenna and has no heading output, while
   `ekf_global` fuses `/gnss/heading` as its only absolute-yaw source. It is
   synthesised from ground truth.

`heading_to_imu` still starts as part of `global_localization.launch.py`, but
stays silent in sim — its UM982 input topic does not exist.

## The M3 forward safety brake

`safety:=true` splices `scan_safety` into the command path, exactly as on the
robot:

```
teleop ──/cmd_vel_raw──▶ scan_safety ──/cmd_vel──▶ gz DiffDrive
                             ▲
                           /scan
```

**You must drive via `/cmd_vel_raw`**, or you publish straight to the chassis
and bypass the brake with no warning:

```bash
ros2 launch outdoor_patrol_sim sim.launch.py safety:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

The node reads **raw scan angles and does not use TF**, so its
`forward_offset_deg: 180.0` depends on the LiDAR being mounted yaw-180. That
holds in sim for the same reason it holds on the robot: the `gpu_lidar` hangs
off `lidar_link`, which `chassis.yaml` yaws by pi. The parameters are used
unmodified — there is no sim-specific safety config.

Measured in the default world, driving at the box at `x=5.0` (front face
`x=4.70`) for 25 s at 0.5 m/s:

| | Final `base_link` x | Gap to box |
| --- | --- | --- |
| `safety:=false` | 4.160 | 0 — robot front at 4.70, hard contact |
| `safety:=true` | 3.774 | 0.39 m of clear air |

The brake fires at exactly `stop_distance_m` (0.5 m from the LiDAR) and the
robot then coasts ~8 cm, because the DiffDrive plugin's
`max_linear_acceleration` limits the deceleration. It blocks forward motion
only — rotation in place still works, so you can turn away and drive off.

## Known differences from the real robot

- **No `/cmd_vel` watchdog.** The gz DiffDrive plugin drives on the last
  command forever; the firmware fails safe and stops. Do not use the sim to
  validate stop-on-signal-loss.
- **No RTK / NTRIP state machine.** The fix is always "RTK-fixed" quality, so
  the sim cannot exercise fix degradation, correction dropout, or the
  covariance-inflation path in `confidence_gate`.
- **Idealised wheel odometry.** No slip, no encoder quantisation, no VESC
  dynamics — dead-reckoning error is far smaller than in the field.
- **The IMU reports orientation**, unlike the real debug unit which emits an
  identity quaternion. `ekf_global` fuses yaw rate only, so this makes no
  difference to the filter, but do not start relying on it.
- **Real-time factor is not 1.0.** On software rendering it runs at roughly
  1.0 headless but drops to ~0.3 with `gui:=true`. Anything you time with a
  wall-clock `sleep` or a `ros2 topic pub` duration will move the robot much
  less than you expect — measure against `/clock` or `/odom_truth`, not a
  stopwatch.

## Headless rendering

The `gpu_lidar` needs a GL context even with no GUI. The launch file sets
`LIBGL_ALWAYS_SOFTWARE` explicitly, because this dev container presets it to
`1` and gz-sim's headless EGL path then aborts with *"Not allowed to force
software rendering when API explicitly selects a hardware device"*. On a host
with no `/dev/dri` render node, pass `software_rendering:=true`.
