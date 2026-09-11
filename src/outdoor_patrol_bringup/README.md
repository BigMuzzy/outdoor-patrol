# Hardware bringup and replacement profiles

The combined [hardware launch](launch/gnss_localization.launch.py) owns the
chassis agent, robot description, localization, sensor startup delays and the
lidar filter/brake. A sensor **profile** is an ordinary ROS Python launch file
that owns its vendor driver and any protocol/message adapters. Changing a
profile does not require editing the combined launch or downstream consumers.

## Stock profiles

| Role | Profile | Default parameters |
|---|---|---|
| GNSS | [gnss_um982.launch.py](launch/profiles/gnss_um982.launch.py) | [UM982 rover](../um982_driver/config/um982_rover.yaml), plus the NTRIP package example unless real credentials are supplied |
| IMU | [imu_inertial_labs.launch.py](launch/profiles/imu_inertial_labs.launch.py) | [imu_driver.yaml](../imu_driver/config/imu_driver.yaml) |
| Lidar | [lidar_rplidar_c1.launch.py](launch/profiles/lidar_rplidar_c1.launch.py) | [lidar.yaml](config/lidar.yaml) |

The first two reuse the drivers' existing lifecycle launches. The IMU profile
turns off its standalone TF/RViz publishers: robot state publisher owns the
mount. The lidar profile publishes `/scan_raw`; it does not own the shared
filter or brake. IMU startup remains delayed 12 s and lidar/filter/brake 8 s.
`use_imu:=false` and `use_lidar:=false` skip the corresponding entire group,
including profile loading. There is no GNSS-disable flag in this launch.

Profile names describe checked-in software, not a verified hardware inventory.
In particular, the IMU profile selects the Inertial Labs binary protocol
implemented by `imu_driver`; it does not establish compatibility with every
unit mentioned in historical chassis comments.

## Selection and override precedence

```bash
ros2 launch outdoor_patrol_bringup gnss_localization.launch.py \
  imu_launch_file:=/absolute/imu_replacement.launch.py \
  imu_params_file:=/absolute/imu_replacement.yaml \
  imu_dev:=/dev/op-imu \
  use_rviz:=false
```

The three `*_launch_file` arguments require **absolute paths** and default to
the installed stock profiles. Missing enabled profiles fail at launch;
disabled IMU/lidar profiles are not resolved. These are trusted executable
Python files, not untrusted data.

Empty `*_params_file` means **the selected profile's default**, not the old
vendor's YAML. A non-empty path replaces that default file; copy the stock
file first if changing only a few fields. ROS YAML node selectors must match
the selected driver's node name/namespace. Stock launches retain support for
legacy relative parameter paths, but absolute paths avoid CWD surprises.

Empty `*_dev` keeps the profile's transport configuration. Non-empty values
override the port after loading its YAML. A profile maps the generic `port`
input to its driver's native parameter (`port`, `serial_port`, etc.).
The stock profiles fail explicitly on missing parameter files.

On the ROS CLI, **omit** an argument to use its default: `name:=` is invalid
CLI syntax, even though an empty launch-configuration value is valid internally.
`GNSS_PORT`/`IMU_PORT`/`LIDAR_PORT` and `GNSS_PARAMS`/`IMU_PARAMS`/`LIDAR_PARAMS`
provide environment-backed defaults. Compose passes optional/empty values
through those environment variables, not malformed empty CLI arguments.
Explicit non-empty CLI arguments override the environment defaults.

| Combined-launch argument | Profile input |
|---|---|
| `gnss_dev`, `imu_dev`, `lidar_dev` | `port`, inside the selected role only |
| `gnss_params_file`, `imu_params_file`, `lidar_params_file` | `params_file` |
| `gnss_auto_activate`, `imu_auto_activate` | `auto_activate`, where lifecycle is supported |
| `imu_baud` | IMU `baud` |
| `ntrip_params_file`, `rtcm_topic` | GNSS correction-source configuration |

Each profile runs in a non-forwarding scoped group. `sensor_role`,
`ros_namespace` and `use_sim_time` are also supplied explicitly; other
sibling/private launch arguments do not leak into it. Stock driver clock
behavior is unchanged; ROS parameters still belong to each profile.
The shared localization launch retains its existing datum/EKF options.

Compatibility: `um982_params_file` is an alias for `gnss_params_file` (an explicit
`gnss_params_file` wins). Legacy `auto_activate` now controls GNSS only;
`imu_auto_activate` is independent. Legacy `baud` remains an alias for `imu_baud`.

## Contract for a replacement

Do not merely rename a topic with incompatible data. Keep these root-namespace
interfaces, or add a normalization adapter inside the new profile:

| Interface | Required meaning |
|---|---|
| `/um982_driver/fix` (`sensor_msgs/msg/NavSatFix`) | Position of the configured primary antenna reference, `gnss_link`; meaningful fix status and ENU position covariance. The confidence gate and mission use covariance to judge quality: zero/unknown covariance must not masquerade as high-confidence RTK. |
| `/um982_driver/heading` (`geometry_msgs/msg/QuaternionStamped`) | Dual-antenna **baseline ENU yaw**, not compass degrees or course over ground. The existing heading adapter separately applies the antenna-baseline-to-body offset. |
| `/rtcm` (`rtcm_msgs/msg/Message`) | Deliver correction bytes to the selected receiver when using the shared correction interface. |
| `/um982_driver/nmea_sentence` (`nmea_msgs/msg/Sentence`) | Live rover NMEA/GGA for NTRIP VRS upload. A replacement GNSS profile owns its correction/GGA loop, whether reusing the current NTRIP client or providing an equivalent source. |
| `/imu_driver/data` (`sensor_msgs/msg/Imu`) | Sensor axes represented by `imu_link`, angular velocity in rad/s, valid covariance and timestamps. The global EKF currently fuses gyro yaw rate only; changing make/model must not silently change axis/sign conventions. |
| `/scan_raw` (`sensor_msgs/msg/LaserScan`) | Meters/radians, correct angle direction/order and timestamps, `lidar_link` consistent with the robot description. The shared filter produces `/scan`. |

The historical `/um982_driver/*` and `/imu_driver/data` names are retained as
compatibility interfaces; the replacement node need not keep the old vendor's
name. Check QoS against consumers, not just message type and spelling.

Profiles must not add a second `map -> odom`, `odom -> base_link`, or sensor
mount broadcaster. Do not move the shared scan filter/brake into the profile,
publish directly to `/cmd_vel`, or disable `use_lidar` to replace its driver.
The common path remains `/cmd_vel_raw -> scan_safety -> /cmd_vel`.

## Calibration is separate from identity and protocol

A replacement at the same mount with the same protocol may need only a new
host identity binding. A new make/model may also require:

- [chassis.yaml](config/chassis.yaml): measured sensor pose/axes and GNSS
  antenna reference. It remains the robot description's geometry source.
- [heading_to_imu.yaml](../outdoor_patrol_loc/config/heading_to_imu.yaml):
  GNSS baseline-to-body offset (currently `-pi/2` for the lateral baseline).
- [scan_safety.yaml](../outdoor_patrol_safety/config/scan_safety.yaml):
  raw scan bearing of robot-forward (currently 180 degrees), range and age
  limits. This raw-angle brake does **not** infer that bearing from TF.
- [scan_box_filter.yaml](config/scan_box_filter.yaml): body mask if body geometry
  changes. Check that the new sensor does not mask genuine obstacles.
- Driver YAML: baud/output mode, covariance, rate/decimation and handedness.

Remounting is not just a USB-path change. Packaged calibration-file edits need
an image rebuild or an explicit bind mount of the changed installed file.

## Deployment and verification

See the [deployment guide](../../deploy/README.md#selecting-driver-profiles)
for mounted profiles/configs and the network-lidar Compose overlay. A profile
can select only packages/executables actually installed in the image. Adding
a new driver/adapter package requires declaring/installing its dependencies
and rebuilding; editing an already-mounted profile or YAML does not.

With motion inhibited, check the new driver's lifecycle/data liveness, types,
QoS, timestamps, covariance and measured rate; then validate axes/heading,
sensor TF and scan placement. For GNSS, check correction delivery and GGA
upload as well as actual fix/heading quality. For lidar, check forward-obstacle
and stale-scan stopping against the configured limits. The existing brake
only blocks forward motion; this work does not turn it into an all-direction
collision or hardware emergency-stop system.

```bash
colcon test --packages-select outdoor_patrol_bringup
```

The package tests expand real ROS launch descriptions without starting drivers.
They pin the stock graph and exercise a deliberately fake alternate vendor
profile. They prove profile/parameter wiring and preservation of shared nodes,
not physical compatibility or achieved sensor performance.
