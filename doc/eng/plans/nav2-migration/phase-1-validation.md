# Phase 1 validation

Run this first:

```bash
colcon build --merge-install --packages-select outdoor_patrol_nav
```

~2 minutes. It has never been compiled — expect to fix something here.

## Steps

1. Build. ~2 minutes.

   ```bash
   colcon build --merge-install --packages-select outdoor_patrol_nav \
     && source install/setup.bash
   ```

2. Unit tests — the route reader and the subsampler. ~1 minute.

   ```bash
   colcon test --packages-select outdoor_patrol_nav
   colcon test-result --verbose
   ```

   `uncrustify` and `cpplint` run here too and have never seen this code.
   Formatting complaints are not failures of the logic; fix them with
   `ament_uncrustify --reformat src/outdoor_patrol_nav` and move on.

3. R3-N, the clean lap. ~15 minutes. `0.129` is the measured Phase 0 baseline
   (mean R3 RMS 0.0645 m × 2) and is now also the default in
   `run_validation.sh`, so the assignment below is belt-and-braces.

   ```bash
   NAV_MAX_RMS=0.129 src/outdoor_patrol_sim/scripts/run_validation.sh \
     /tmp/val teach r3n
   ```

4. R5-N, the degraded fix. ~5 minutes, reuses the route from step 3.

   ```bash
   src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/val r5n
   ```

5. Paste both JSON files back. ~1 minute.

   ```bash
   cat /tmp/val/score_R3-N.json /tmp/val/score_R5-N.json
   ```

## Pass criteria

**R3-N** — all five must hold:

| Field | Gate |
|---|---|
| `cross_track_rms_m` | < `NAV_MAX_RMS` |
| `cross_track_max_m` | < 0.50 m |
| `laps` | ≥ 0.98 |
| `longest_stop_s` | < 3.0 s |
| `lateral_min_m` / `lateral_max_m` | inside the corridor |

**R5-N** — both must hold:

| Field | Gate |
|---|---|
| `degraded_cycles` | ≥ 20 |
| `final_speed_ms` | ≤ 0.001 |

## If it fails

Five failures ranked by how likely they are, first the two that mean "it never
started":

**The robot never moves, no error anywhere.** Cause: message type. Check
`ros2 topic info /cmd_vel_raw` — it must be `geometry_msgs/msg/Twist`, not
`TwistStamped`. Fix: `enable_stamped_cmd_vel: false` is missing from one server
in `config/nav2_params.yaml`.

**`waiting for the navigate_through_poses action server`, forever.** Cause:
`bt_navigator` did not activate, almost always a configure-time parameter
error that took the whole file with it. Fix: read the first error in
`/tmp/val/sim_R3-N.log`, then:

```bash
ros2 lifecycle get /bt_navigator
```

**`waiting for /fromLL`.** Cause: `navsat_transform` is not running, or
`localization:=false`. Fix: `ros2 service list | grep fromLL`.

**`no /patrol_mission/status`.** Cause: the mission node died at start-up.
Fix: read `/tmp/val/follower_R3-N.log` — the first line names the route file
problem.

**Fails only on `longest_stop_s`.** Cause is probably the tail of the bag, not
the controller. Check risk 2 in [progress.md](./progress.md) before changing
any parameter:

```bash
ros2 bag play /tmp/val/bag_r3n --topics /cmd_vel  # or read the tail directly
```

## Do not

- Do not take this outdoors until both runs pass. `sim.launch.py nav:=true`
  and the robot's bringup share node names.
- Do not tune MPPI critics here. That is Phase 2, after the corridor is a
  costmap filter — tuning against a corridor the planner cannot see produces
  parameters that stop being right the moment it can.

## Next

Run step 1 now:
`colcon build --merge-install --packages-select outdoor_patrol_nav`.
