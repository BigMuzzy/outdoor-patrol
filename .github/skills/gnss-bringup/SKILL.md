---
name: gnss-bringup
description: "Use when the user asks to bring up / test the GNSS global-localization stack on the robot, run a GNSS/RTK field test, get an RTK fix with NTRIP credentials, or visualize GNSS localization in RViz on the host. Covers standalone vs RTK, host RViz, and fix/TF/odometry verification. DO NOT USE for plain teleop only (see connect-to-robot/driving), building/pushing the deploy image (deploy-to-robot), or VESC tuning."
---

# GNSS global-localization bringup & test

Brings up the interim GNSS global localization (ADR-012) on the robot and
visualizes it in RViz on the dev box. The deployed container already runs this
stack (`outdoor_patrol_bringup gnss_localization.launch.py use_rviz:=false`),
which composes: micro-ROS agent + `robot_state_publisher` → UM982 RTK GNSS +
NTRIP (`gnss_rtk.launch.py`) → dual-EKF + `heading_to_imu` + `confidence_gate`
+ `navsat_transform` (`global_localization.launch.py`).

Prereqs: robot reachable (`ssh robot`, see **connect-to-robot**); deploy stack
running (see **deploy-to-robot**). UM982 = `/dev/ttyUSB0`
(`usb-1a86_USB_Serial-if00-port0`), mapped into the container by its by-id path.
Host & robot are both Cyclone DDS, `ROS_DOMAIN_ID` unset (=0), host networking,
so the dev box discovers the robot's topics directly.

## Standalone vs RTK

| Mode | Corrections | `/um982_driver/fix` `status.status` | Position cov |
|---|---|---|---|
| Standalone (SPS) | none | `0` (STATUS_FIX) | ~1.3 m² |
| SBAS/DGPS | SBAS | `1` (STATUS_SBAS_FIX) | sub-m |
| **RTK float/fix** | NTRIP | **`2` (STATUS_GBAS_FIX)** | float ~0.09 m², **fix ~0.0004 m²** |

The driver maps GGA quality 4/5 (RTK fix/float) → `STATUS_GBAS_FIX` and shrinks
covariance (RTK-fix std 0.02 m). So **RTK is confirmed by `status: 2` + tiny
covariance**, not a distinct enum.

## Enable RTK with real NTRIP credentials (outdoor test)

`ntrip.yaml` is gitignored (`**/ntrip.yaml`); never commit real caster creds.
The dev box keeps the real one at repo-root `ntrip.yaml`.

1. Copy it to the robot's compose data volume (`./data` → `/data` in-container):
   ```bash
   ssh robot 'mkdir -p ~/code/outdoor-patrol/deploy/data'
   scp ntrip.yaml robot:~/code/outdoor-patrol/deploy/data/ntrip.yaml
   ```
2. Start the stack with the `NTRIP_PARAMS` env knob (wired into the compose
   `command:`; unset → the in-image example = standalone):
   ```bash
   ssh robot 'cd ~/code/outdoor-patrol && NTRIP_PARAMS=/data/ntrip.yaml \
     docker compose -f deploy/docker-compose.yaml up -d'
   ```
3. NTRIP needs a live GGA upload, so it only locks once the receiver has its
   own fix outside with sky view. Expect float → fix over tens of seconds.

ntrip.yaml fields: `host`, `port`, `mountpoint`, `username`, `password`,
`ntrip_version`, `send_gga: true`, `gga_period_s`. See
[src/ntrip_client/config/ntrip.yaml.example](../../../src/ntrip_client/config/ntrip.yaml.example).

## RViz on the host

```bash
cd /workspaces/outdoor-patrol && source install/setup.bash
rviz2 -d src/outdoor_patrol_bringup/config/gnss.rviz
```

Fixed frame = `map`. The `Failed to parse type hash …` Cyclone warnings are
benign (micro-ROS DDS participant). Close the window when done.

## Verify (from the dev box)

```bash
source install/setup.bash
# Fix quality + covariance (RTK = status 2, cov ~0.0004)
ros2 topic echo --once /um982_driver/fix | grep -A2 status
ros2 topic echo --once /um982_driver/fix | grep -A10 position_covariance
# Rates: fix/heading ~5 Hz, global EKF ~30 Hz
ros2 topic hz /um982_driver/fix
ros2 topic hz /odometry/global
# TF chain must connect end-to-end
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
# Global pose (frame_id: map, child_frame_id: base_link)
ros2 topic echo --once /odometry/global
```

Known-good baseline (stationary, standalone): TF `map→odom→base_link`
connected; `/odometry/global` ~30 Hz; fix/heading ~5 Hz.

## Heading caveat (finalize in the field)

`heading_to_imu` `yaw_offset` is still the **assumed** `-pi/2` for the lateral
ANT1-right/ANT2-left baseline (see
[src/outdoor_patrol_loc/config/heading_to_imu.yaml](../../../src/outdoor_patrol_loc/config/heading_to_imu.yaml)).
Finalize empirically: point the nose along a known heading, compare
`/gnss/heading` (or EKF yaw) to truth; if ~180° off, use `+pi/2`. Also verify
`/gnss/heading` is actually fused in
[ekf_global.yaml](../../../src/outdoor_patrol_loc/config/ekf_global.yaml)
(`imu0` + `_config`) — a flat ~0 yaw in `/odometry/global` is the symptom of it
not being fused.

## Driving during the test

The deployed stack includes teleop. Drive from the host:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.4
```
Chassis failsafe (firmware `rc_failsafe.c`): RC mode switch >1500 µs = MANUAL
(sticks); else `/cmd_vel` fresh (<500 ms) = AUTONOMOUS; else FAILSAFE_STOP
(`/failsafe/active: true`). Release keys → 500 ms auto-stop. **RC TX manual
switch is the override** — confirm the area is clear before enabling motion.

## Outdoor field-test checklist

- [ ] Clear sky view; start near the dock (datum is auto-on-first-fix, `navsat.yaml`).
- [ ] Real `ntrip.yaml` on the robot; confirm RTK (`status: 2`, cov ~0.0004).
- [ ] RC TX on and in reach (override); area clear; e-stop ready.
- [ ] RViz `map` frame on host shows fix + `/odometry/global` tracking.
- [ ] Heading sanity-checked against a known direction; adjust `yaw_offset` if needed.

## Troubleshooting

- **No `/um982_driver/fix`** — check the device is mapped in the container
  (`docker exec outdoor-patrol ls /dev/serial/by-id/`) and the lifecycle node
  is active (`/um982_driver/transition_event`, node reaches `active`).
- **Stuck at `status: 0` outdoors** — NTRIP not connected: check
  `docker compose logs` for `ntrip_client` auth/host errors; verify
  mountpoint/credentials and that GGA upload is happening.
- **Host can't see topics** — both ends must be Cyclone DDS, same
  `ROS_DOMAIN_ID`; WiFi APs sometimes filter multicast.
- **`map` jumps** — expected on standalone; tightens with RTK + fused heading.
