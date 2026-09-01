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

# Terminal 1: headless sim + dual-EKF + GNSS + forward safety brake
ros2 launch outdoor_patrol_sim sim.launch.py

# Terminal 2: keyboard teleop. The brake sits between /cmd_vel_raw and
# /cmd_vel, so the remap is REQUIRED -- without it you drive the chassis
# directly and bypass the brake. Hold the key to keep moving: a single
# keypress expires after 500 ms (see "command TTL" below).
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw

# Terminal 3 (optional): RViz, ground truth overlaid on the EKF estimate
ros2 launch outdoor_patrol_sim sim.launch.py use_rviz:=true
```

> **Run only ONE sim at a time.** Two `gz sim` servers share the same
> gz-transport topic names, so a second instance silently steals `/cmd_vel`
> and `/scan` from the first. If the robot ignores the brake or moves when
> you did not tell it to, check `pgrep -af "gz sim server"` first.

Useful arguments:

| Argument | Default | Effect |
| --- | --- | --- |
| `gui` | `false` | `true` opens the Gazebo GUI (needs a display). |
| `use_rviz` | `false` | RViz with [`config/sim.rviz`](config/sim.rviz), fixed frame `map`. |
| `localization` | `true` | `false` skips the dual-EKF; no `map`/`odom` TF. |
| `gnss` | `true` | `false` skips `gnss_sim`; no fix, no heading. |
| `safety` | `true` | M3 forward brake between `/cmd_vel_raw` and `/cmd_vel`. `false` gives the ungated path. |
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

On by default (`safety:=true`), matching the robot, where the brake is always
in the command path. `scan_safety` is spliced in ahead of the chassis:

```
teleop ──/cmd_vel_raw──▶ scan_safety ──/cmd_vel──▶ gz DiffDrive
                             ▲
                           /scan
```

**Drive via `/cmd_vel_raw`.** Publishing to `/cmd_vel` reaches the chassis
without passing the brake — silently, with no error. This is not a sim quirk;
the real robot's `gnss_localization.launch.py` wires `scan_safety` the same
way, so the same bypass exists there.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

Confirm the brake is actually live before trusting a run:

```bash
ros2 node list | grep scan_safety           # must print exactly one
pgrep -af "gz sim server"                   # must print exactly one
ros2 param get /ekf_filter_node odom0       # must print /odom, not "not set"
```

The node reads **raw scan angles and does not use TF**, so its
`forward_offset_deg: 180.0` depends on the LiDAR being mounted yaw-180. That
holds in sim for the same reason it holds on the robot: the `gpu_lidar` hangs
off `lidar_link`, which `chassis.yaml` yaws by pi. The parameters are used
unmodified — there is no sim-specific safety config.

Measured in the default world, driving at the box at `x=5.0` (front face
`x=4.70`) for 25 s at 0.5 m/s with a continuously refreshed command:

| | Final `base_link` x | Gap to box |
| --- | --- | --- |
| `safety:=false` | 4.160 | 0 — robot front at 4.70, hard contact |
| `safety:=true` | 3.774 | 0.39 m of clear air |

The brake fires at exactly `stop_distance_m` (0.5 m from the LiDAR) and the
robot then coasts ~8 cm, because the DiffDrive plugin's
`max_linear_acceleration` limits the deceleration. It blocks forward motion
only — rotation in place still works, so you can turn away and drive off.

### Why it re-gates on a timer

The brake holds the last raw command and re-evaluates it against the freshest
scan every `control_period_s` (20 Hz), rather than gating each command as it
arrives. Arrival-time gating cannot stop a robot that is *already* moving, and
that is exactly the sim's teleop case: `teleop_twist_keyboard` publishes one
Twist per keypress, the DiffDrive plugin holds it forever, and no later
message ever arrives to gate against an obstacle that appears afterwards — the
robot drove straight into the box with the brake running and reporting no
fault.

A held command expires after `cmd_timeout_s` (0.5 s, mirroring
`CMD_VEL_TIMEOUT_MS` in the firmware's `rc_failsafe.h`). On expiry the node
emits a single zero Twist and then goes quiet, so a dead command source stops
the robot here while the firmware watchdog still sees silence and fails safe.
The visible consequence in sim: **one keypress drives for ~0.5 s and stops**
— hold the key to keep going, exactly as on the robot.

## Known differences from the real robot

- **No chassis-level `/cmd_vel` watchdog.** The gz DiffDrive plugin drives on
  the last command forever. In the default configuration `scan_safety` covers
  for it (see the command TTL above), but with `safety:=false` nothing stops
  the robot, and either way the sim exercises the brake's stand-in — not the
  firmware's own stop-on-signal-loss path.
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

## Launch gotchas worth knowing

Two non-obvious things this launch file works around. Both cost real debugging
time; do not "simplify" them away.

**Scoped includes.** `IncludeLaunchDescription` does *not* create a scope, so a
`DeclareLaunchArgument` inside an included file leaks its value into the parent
context. `scan_safety.launch.py` and `localization.launch.py` both declare an
argument called `params_file`. Unscoped, the safety include leaks
`scan_safety.yaml`; the EKF include then sees `params_file` already set, skips
its own default, and loads the *safety* YAML instead of `ekf.yaml`. The node
starts, advertises `/odometry/filtered` and `/tf`, and then publishes nothing
at all — because it has no `odom0` input. Every include here is therefore
wrapped in `GroupAction(scoped=True)`.
[`gnss_localization.launch.py`](../outdoor_patrol_bringup/launch/gnss_localization.launch.py)
hit the same trap and solves it by passing `params_file` explicitly.

**The `/clock` gate.** `robot_localization` calls
`Clock::wait_until_started()` inside `initialize()`, before its executor spins.
Start it before Gazebo is publishing `/clock` and it parks on "Waiting for
clock to start...". How long Gazebo takes to load the world varies with the
machine and the sensor count, so a fixed delay is not reliable; the launch
instead blocks on the first real `/clock` message and starts the EKF stack
after that (plus `localization_start_delay` of margin).

## Headless rendering

The `gpu_lidar` needs a GL context even with no GUI. The launch file sets
`LIBGL_ALWAYS_SOFTWARE` explicitly, because this dev container presets it to
`1` and gz-sim's headless EGL path then aborts with *"Not allowed to force
software rendering when API explicitly selects a hardware device"*. On a host
with no `/dev/dri` render node, pass `software_rendering:=true`.
