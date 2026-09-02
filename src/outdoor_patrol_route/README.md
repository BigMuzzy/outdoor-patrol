# outdoor_patrol_route

GNSS teach-and-repeat: drive a route once, then repeat it autonomously,
stepping onto the shoulder to get around whatever is in the way.

Design and validation results: [issue #8](https://github.com/BigMuzzy/outdoor-patrol/issues/8)
and [doc/eng/plans/issue-8-teach-and-repeat.md](../../doc/eng/plans/issue-8-teach-and-repeat.md).

Two nodes, and a route file between them:

```
  teach   /odometry/global ──► route_recorder ──► route.yaml
  repeat  route.yaml ──► route_follower ──► /cmd_vel_raw ──► scan_safety ──► /cmd_vel
```

## The one idea

Retreat is not "go somewhere else". It is "follow the same path, shifted
sideways":

```
  look_ahead = centerline(s + L) + d * normal(s + L)
```

`d = 0` is normal patrol. `d = -2.4 m` is parked on the right shoulder. Ramping
`d` between them is a lane change. Retreat and resume are the same operation
with opposite sign, so there is no second control mode, no state machine and no
costmap.

Choosing `d` is likewise not a special case. Every cycle the follower asks, for
each candidate offset from zero outward, "is the corridor at this offset clear
for the next few metres?", and takes the smallest `|d|` that is. That one rule
gives all the required behaviour — drive the centerline when clear, step out
when blocked, come back when it clears, and **stop in place** when nothing is
clear.

Hysteresis is asymmetric on purpose: moving further out happens on the first
cycle that demands it, coming back needs `resume_clear_cycles` consecutive
clear cycles. One dropped return should not start the offset oscillating.

## Record base_link, not the antenna

`gnss_link` sits **0.28 m forward and 0.42 m right** of `base_link`
([chassis.yaml](../outdoor_patrol_bringup/config/chassis.yaml)). Record the raw
fix and your "centerline" is 0.42 m to the right of where the robot actually
drove — and through a corner the antenna sweeps an arc while `base_link` barely
translates, which no constant offset can describe.

`source` says which correction produced a file, and the follower refuses to
follow one that has none:

| `source` | Position from | base_link? |
|---|---|---|
| `odometry_global` | `/odometry/global` (navsat_transform applies the TF lever arm) | yes |
| `fix_lever_arm` | gated fix − R(yaw)·t_antenna, computed here | yes |
| `raw_antenna` | gated fix, uncorrected | **no** — differential-test control only |

Measured in simulation against Gazebo ground truth: both corrected paths land
within 0.10 m of the true centerline, the control at −0.42 m. If all three
scored the same, the correction would not be wired — which is the whole point
of keeping the control.

## Route file

Geodetic, not `map` XY: `navsat_transform` auto-sets its datum on the first fix
unless pinned, so a `map`-frame track silently shifts between sessions. The
datum in force at recording time is stored alongside, and the file is projected
back to `map` at load time through `fromLL`.

```yaml
version: 1
recorded: 2026-09-02T01:28:05+00:00
source: odometry_global
frame: base_link
loop: true
lane_half_width_m: 2.000
datum: {latitude: -41.286460000, longitude: 174.776236000, altitude: 0.000}
samples:
  - {lat: ..., lon: ..., alt: ..., yaw: ..., fix: fixed, sigma_h: 0.020,
     shoulder_left_m: 1.00, shoulder_right_m: 1.00}
```

`loop: true` is verified, not taken on trust: the recorder measures the gap
between the first and last station and writes `false` if the pass did not
actually come back. Per-sample shoulder widths are written even at a constant,
so multi-pass shoulder measurement can fill them in later without a format
change. Samples below the fix threshold are **flagged, never dropped** — a
dropped sample is the only evidence that a stretch of route is GNSS-marginal.

## Use it

Bring the GNSS stack up first; neither launch file starts one.

```bash
# Teach. Drive the route, then save.
ros2 launch outdoor_patrol_route route_record.launch.py output_path:=/tmp/route.yaml
ros2 service call /route_recorder/save std_srvs/srv/Trigger

# Repeat.
ros2 launch outdoor_patrol_route route_follow.launch.py route_path:=/tmp/route.yaml
```

Sampling triggers on **1 m of travel or 5° of yaw change** — a timer
under-samples corners and over-samples straights.

`/route_follower/status` carries JSON: state, station, cross-track, commanded
offset, which candidate offsets are blocked, speed and fix sigma. It is a
`std_msgs/String` rather than a custom message so the validation harness can
bag and parse it without a message package; if it outgrows that, promote it.

## The parameter pair that will bite you

`trigger_range_m` and `ramp_lateral_per_m` are a pair, and getting the pair
wrong deadlocks the robot silently.

The ramp is parameterised by **distance travelled**, so reaching the outermost
offset always costs `|d| / ramp` metres no matter how slowly you drive. On top
of that the vehicle trails the commanded offset — measured at ~3 m on this
chassis, because pure pursuit steers toward a point 1.2–4 m ahead and the yaw
rate is capped. Both have to fit inside the warning distance:

```
  ramp 0.6 m/m -> 2.4 m of offset in 4.0 m
  + ~3 m of vehicle lag             = ~7 m
  trigger 10.0 m                    -> ~3 m spare
```

Get it wrong and the robot arrives at the obstacle still out of position.
`scan_safety` then zeroes forward velocity at 0.5 m — correctly — but pure
pursuit derives yaw rate from forward speed, so it can no longer steer out, and
the distance-parameterised ramp stops advancing too. Stuck, safely, forever.
This was observed, not theorised; `route_follower` now checks the pair at
start-up and logs an error if it cannot work.

## Tests

```bash
cd src/outdoor_patrol_route && PYTHONPATH=. python3 -m pytest test/ -q
```

Covers the schema round-trip and its rejection paths, and the path geometry
against a closed-form circle. The end-to-end behaviour is validated in
simulation by
[`run_validation.sh`](../outdoor_patrol_sim/scripts/run_validation.sh).
