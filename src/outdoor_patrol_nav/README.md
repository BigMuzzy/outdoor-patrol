# outdoor_patrol_nav

Nav2 for the outdoor patrol robot. Replaces the custom pure-pursuit
`route_follower` with the stock Nav2 stack, keeping the recorded
teach-and-repeat route as the source of goals.

This package is **Phase 1** of
[`doc/eng/plans/nav2-migration/`](../../doc/eng/plans/nav2-migration/plan.md).
Read [`progress.md`](../../doc/eng/plans/nav2-migration/progress.md) before
changing anything here — it records what has been validated and what has only
been written.

## Role in the stack

```
route_*.yaml ─┐
              ├─▶ patrol_mission ──NavigateThroughPoses──▶ bt_navigator
  /fromLL  ───┘        │                                        │
 (navsat_transform)    │                              planner_server (Hybrid-A*)
                       │                              smoother_server (Sav-Golay)
                       │                              controller_server (MPPI)
                       │                                        │
                       │                                  /cmd_vel_nav
                       │                                        │
                       │                              velocity_smoother
                       │                                        │
                       │                                  /cmd_vel_raw
                       ▼                                        │
                 ~/status, ~/finished          scan_safety ──▶ /cmd_vel ──▶ wheels
```

`/cmd_vel` is `scan_safety`'s **output**, not Nav2's. Every Nav2 server that
publishes velocity is remapped to `/cmd_vel_nav` in `launch/nav2.launch.py` so
the M3 forward brake stays between the planner and the wheels (ADR-013). Two
writers on `/cmd_vel` is the failure mode this remapping exists to prevent.

## What is custom, and why

Almost nothing, on purpose. One node, `src/patrol_mission.cpp`, covering the
three things Nav2 has no way to know:

| | |
|---|---|
| The route is **geodetic** | Every sample is projected through `robot_localization`'s `/fromLL` at start-up, so the mission survives a datum change. |
| Nav2 has no opinion about **GNSS quality** | Above `sigma_stop_m` on the raw driver fix the goal is cancelled; below `sigma_slow_m` for `resume_clear_cycles` it is re-sent from where it stopped. |
| The harness parses a **status topic** | `~/status` mirrors `/route_follower/status`, so `score_run.py` needs only `--status-topic`. |

Everything else is stock: Hybrid-A* plans, `SavitzkyGolaySmoother` smooths,
MPPI drives, `velocity_smoother` clamps, `behavior_server` recovers.

Two design choices that look wrong until you know why:

- **`NavigateThroughPoses`, not `FollowGPSWaypoints`.** The GPS action would
  do the geodetic projection for us, but it runs one `NavigateToPose` per
  waypoint and comes to a **stop** at each. With stations every 10 m that is a
  stop every 10 m, and R3-N requires the longest stop to stay under 3 s.
- **`patrol_mission` publishes no velocity, on any topic.** On a degraded-GNSS
  cancel, Nav2 stops commanding and `scan_safety`'s `cmd_timeout_s` (0.5 s)
  emits the zero `Twist` that actually stops the robot — the same path the
  follower relies on. Publishing our own zero would put a second writer on
  `/cmd_vel_raw`.

## Run

Servers only (no mission):

```bash
ros2 launch outdoor_patrol_nav nav2.launch.py use_sim_time:=true
```

With the mission, against a recorded route:

```bash
ros2 run outdoor_patrol_nav patrol_mission --ros-args \
  --params-file $(ros2 pkg prefix outdoor_patrol_nav)/share/outdoor_patrol_nav/config/patrol_mission.yaml \
  -p use_sim_time:=true \
  -p route_path:=/tmp/val/route_odometry_global.yaml
```

`--params-file` **first**: ROS 2 applies overrides in order and the file
carries an empty `route_path` that would otherwise win.

In simulation, both come up together:

```bash
ros2 launch outdoor_patrol_sim sim.launch.py nav:=true
# or, scored end to end:
src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/val teach r3 r3n
```

## Parameters worth knowing

`config/patrol_mission.yaml` deliberately mirrors
`outdoor_patrol_route/config/route.yaml`, so R5 and R5-N test the same rule:

| Parameter | Default | Note |
|---|---|---|
| `route_path` | `""` | Required. Refuses `source: raw_antenna`. |
| `fix_topic` | `/um982_driver/fix` | **Raw**, not `/gnss/fix_gated`: the gate multiplies covariance by 1000 on a degraded fix, which would turn the slow/stop pair into a cliff at 5 cm. |
| `station_spacing_m` | `10.0` | Dense enough that Hybrid-A* returns the centerline, sparse enough not to re-plan between neighbours. |
| `sigma_slow_m` / `sigma_stop_m` | `0.10` / `0.50` | Same numbers `confidence_gate` uses. |
| `laps` | `1.0` | Loop routes only; ignored with a warning on an open route. |

`config/nav2_params.yaml` documents its own three "walls, not knobs": the
velocity smoother limits (firmware clamps), the footprint, and
`minimum_turning_radius: 1.5`.

## Tests

```bash
colcon build --merge-install --packages-select outdoor_patrol_nav
colcon test --packages-select outdoor_patrol_nav
colcon test-result --verbose
```

`test/test_route_goals.cpp` covers the route reader and the subsampler — the
only two pure functions here. Everything else needs a running stack and is
covered by `run_validation.sh` R3-N and R5-N.

## Not here yet

`map_server`, the keepout/speed costmap filters and `route_to_map` arrive in
Phase 2; the safe-spot retreat BT nodes in Phase 4; `collision_monitor` in
Phase 6. There is no occupancy map of the site — GNSS is the map.
