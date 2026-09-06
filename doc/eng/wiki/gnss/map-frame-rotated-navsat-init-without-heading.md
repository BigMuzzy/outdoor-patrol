# Map frame silently rotated — `navsat_transform` initialised without a heading

- **Date:** 2026-09-06
- **Affects:** `navsat_transform_node` (`outdoor_patrol_loc`), `ekf_global`,
  RViz `GlobalEKF` display, `outdoor_patrol_validation` Phase 2
- **Severity:** gotcha (silent — every topic looks healthy in isolation)

## Symptom

In RViz the robot **crabbed sideways**: the orange `/odometry/global` arrow
trail walked diagonally across the screen, arrows pointing one way while the
trail advanced in another. On the ground the robot drove straight.

Every individual indicator looked fine at the same time:

```
gga_quality 4 (RTK fixed)   sats 27   hdop 0.5   corr_age 0.6s
heading_ok True   heading_yaw_deg 21.1   odom_yaw_deg 21.1   <- agree exactly
```

Validation Phase 2 failed with a **143.9°** heading error, and its hint
("`yaw_offset` is wrong by roughly this much") pointed at a config value that
turned out to be correct.

Two details made this confusing rather than obvious:

* It **worked in simulation**, so it looked like a field/hardware problem.
* `heading_yaw_deg` and `odom_yaw_deg` matched to the decimal, so the EKF
  looked like it was tracking the GNSS heading perfectly. It was.

## Root cause

`navsat_transform` computes the UTM→map rotation **once**, from the IMU
heading available at the moment it initialises, and never revisits it. In this
stack that input is `/gnss/heading` (remapped in
[`global_localization.launch.py`](../../../../src/outdoor_patrol_loc/launch/global_localization.launch.py)).

The container had started while **ANT2 had no signal**, so `um982_driver` was
correctly gating the invalid dual-antenna heading (see
[heading-wrong-ant2-no-signal.md](heading-wrong-ant2-no-signal.md)) and
`/gnss/heading` was **silent**. The container logs put this beyond doubt — the
datum was computed 34 ms before a heading-drop warning:

```
[1788674908.826] [navsat_transform]: Datum (latitude, longitude, altitude) is (47.54, -121.88, 254.88)
[1788674908.860] [um982_driver]: Dropping GNSS heading: KSXT heading-quality 0 < 1
```

So the map frame was pinned to whatever orientation was available with no valid
heading, and **stayed there for twelve hours**. There is no `datum` service
exposed on this node (`ros2 service list` shows only parameter services), so it
cannot be re-seeded at runtime — the rotation persists until the node restarts.

The result is an inconsistency *between* two inputs of `ekf_global` that is
invisible in either one alone:

| Input | Frame it is actually in |
|---|---|
| `imu0` = `/gnss/heading` (yaw) | true ENU — correct |
| `odom1` = `/odometry/gps` (x, y) | map frame — **rotated** |

The EKF cannot reconcile them. Wheel odometry projected through the yaw pushes
the estimate one way while GNSS position pulls it another, so the fused track
oscillates. Measured over 1 m chords during a straight drive, the course swung
between −64° and −164° while the yaw held steady near −20°:

```
time    yaw     course    crab
  0.0   -13.6    -63.8    +50.2
  7.5   -26.4   -137.4   +111.0
 13.1   -27.2   -164.4   +137.2
 18.7   -20.2    -73.0    +52.8
```

**A steady yaw with a wildly swinging course is the signature.** A robot that
is physically turning moves both together.

Simulation never showed it because the sim publishes a valid heading from
`t=0`, so `navsat_transform` always initialises correctly there.

## Fix

Restart the stack **while the heading is healthy**, so the rotation is computed
from a valid heading:

```bash
# confirm first -- want HdgQual 3 and /gnss/heading publishing
ssh robot 'docker exec outdoor-patrol bash -lc "source /opt/outdoor-patrol/install/setup.bash; \
  timeout 10 ros2 topic echo /um982_driver/nmea_sentence > /tmp/n.txt 2>/dev/null; \
  grep -oE \"KSXT[^*]*\" /tmp/n.txt | cut -d, -f12 | sort | uniq -c; \
  timeout 8 ros2 topic hz /gnss/heading 2>/dev/null | grep -m1 average"'

ssh robot 'docker restart outdoor-patrol'
```

Then confirm the new datum carries no heading drops before it:

```bash
ssh robot 'docker logs --since 5m outdoor-patrol 2>&1 | grep -E "Datum|Dropping GNSS heading"'
```

A healthy restart shows the `Datum` lines and **zero** `Dropping GNSS heading`
warnings.

⚠️ **This recurs on every cold start where the heading is not yet valid** — in
particular a power-on after a battery recharge, when the receiver still has to
reacquire ANT2. Until the launch gates `navsat_transform` on a valid
`/gnss/heading`, treat "restart the stack once the heading is good" as part of
the bring-up procedure, not as a one-off repair.

## How to verify

[`scripts/map_frame_check.py`](../../../../scripts/map_frame_check.py) measures
the rotation directly. Drive straight ~5 m while it captures both
`/odometry/global` and raw `$GNGGA`; the difference between the two courses
**is** the map rotation, and it needs no ground truth:

```bash
ssh robot 'docker exec outdoor-patrol bash -lc \
  "source /opt/outdoor-patrol/install/setup.bash && python3 /data/map_frame_check.py"'
```

Healthy is `MAP ROTATION` within ±5° and `scale ratio` ≈ 1.0.

Two companion scripts separate the possible causes, which matters because the
symptoms overlap:

| Script | Question it settles | Reads |
|---|---|---|
| [`gnss_heading_offset.py`](../../../../scripts/gnss_heading_offset.py) | Is `yaw_offset` (antenna mounting) right? | raw NMEA only |
| [`ekf_crab_check.py`](../../../../scripts/ekf_crab_check.py) | Do the RViz arrows point along the path? | `/odometry/global` |
| [`map_frame_check.py`](../../../../scripts/map_frame_check.py) | Is the map frame rotated vs true ENU? | both |

Run `gnss_heading_offset.py` **first**. It touches neither EKF nor
`yaw_offset`, so it cannot be fooled by this bug — on the run that diagnosed
this it returned −88.83° against a configured −90°, which is what ruled the
antenna mounting out and moved the search to the map frame.

## Why Phase 2 could not diagnose this

Phase 2 takes the course it compares against from `/odometry/global`, which is
partly derived from the heading under test. As a *gate* that is fine — it
correctly went red. As a *measurement* it is circular, and its 143.9° error
was neither the antenna offset nor a number worth acting on: it named
`yaw_offset` as the culprit when `yaw_offset` was correct.

If Phase 2 fails, measure with `gnss_heading_offset.py` before changing
`heading_to_imu.yaml`.

## Related

- [heading-wrong-ant2-no-signal.md](heading-wrong-ant2-no-signal.md) — the ANT2
  failure that leaves `/gnss/heading` silent in the first place
- [rtk-fix-hard-to-hold-while-moving.md](rtk-fix-hard-to-hold-while-moving.md)
- `src/outdoor_patrol_loc/config/navsat.yaml`,
  `src/outdoor_patrol_loc/config/ekf_global.yaml` (`imu0`, `odom1`)
