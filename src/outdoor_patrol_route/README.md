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

## What path shapes can you teach?

Most of them. Measured against the real `Path` and a replay of
`route_follower._pursue`:

| Corner radius | Path reproduces the stations | Pure pursuit tracks within |
|---|---|---|
| 10 m | 0.00 m | 0.03 m |
| 3 m | 0.02 m | 0.09 m |
| 1.5 m | 0.09 m | 0.16 m |
| 1.0 m | 0.06 m | 0.23 m |
| 0.7 m | — | 0.29 m |

Corners down to about a metre, hairpins, closed loops and figure-eights all
work. The recorder's 5° yaw trigger is what makes tight geometry survive: on a
1 m corner it drops a station every 8.7 cm, so the spline has plenty to work
with even though the distance trigger is 1 m.

Three limits are real:

**It never reverses.** The commanded speed is floored at `min_speed_ms` and is
always positive, so the route has to be drivable as one continuous forward
motion. No three-point turns, no backing out of a dead end, no pivoting on the
spot. You can *record* a teach pass that reverses; the follower cannot
reproduce it.

**Corners want to be ≥ 1 m radius.** Pure pursuit cuts across anything much
tighter than its `lookahead_min_m` (1.0–1.2 m). The hard geometric floor from
`min_speed_ms / max_angular_rads` is 0.22 m, but tracking is well outside spec
long before that.

**The retreat lane folds on tight inside corners.** This is the one that is
easy to miss. The retreat is a lateral displacement along the path normal, so
the lane it steers to is the centerline offset by `d` — and on the *inside* of
a bend that offset curve shrinks. Once `|d|` reaches the corner radius it
collapses to a cusp and then inverts, and `offset_at` still returns a point, so
the look-ahead silently jumps behind the robot and pure pursuit steers it into
the corner it was avoiding.

`route_follower` now checks this at start-up, against the route it just loaded:

```
retreat geometry vs route: widest offset -1.20 m against a 4.04 m tightest
                           right-hand corner at s=17.3 m (70% lane left)
```

and refuses one that cannot work:

```
retreat lane is unusable: offsetting -1.20 m inverts the path at s=15.2 m,
  where the route turns with a 0.50 m radius. The look-ahead point would jump
  behind the robot and steer it INTO the corner. Reduce corridor_half_width_m
  below 1.05 m, or re-teach that corner wider.
```

Only the retreat side is checked. Offsets on the outside of a bend stretch
rather than shrink, so they can never fold.

## Retuning while it runs

These are settable with `ros2 param set` on a running `route_follower`, or
from the field dashboard's **Follower (live)** box:

| Parameter | Why you would change it mid-run |
|---|---|
| `avoidance_enabled` | **off = track the taught line only.** For measuring what localization alone can do, with no retreat manoeuvres in the cross-track. |
| `nominal_speed_ms`, `min_speed_ms`, `max_angular_rads` | too fast for the corner you just watched it cut |
| `corridor_half_width_m`, `offset_step_m`, `clearance_half_width_m` | the corridor is wider than the route's tightest corner (offsets are recomputed) |
| `trigger_range_m`, `ramp_lateral_per_m`, `resume_clear_cycles` | retreat starting too late or too abruptly |
| `sigma_slow_m`, `sigma_stop_m`, `fix_timeout_s` | **think hard first** -- these are what stop it driving on a fix it cannot trust |
| `laps`, `retreat_side` | |
| `show_corridor` | **display only** -- hide the lane/corridor bands when folded offsets clutter the view. The centerline and look-ahead point stay. |

Several of these are cached at start-up rather than read every cycle, so the
node re-caches them through a set-parameters callback. Without that,
`ros2 param set` would appear to work and change nothing.

Values are validated: a negative speed or an unknown `retreat_side` is
rejected with a reason rather than accepted and ignored.

`avoidance_enabled: false` does **not** disable the forward safety brake --
`scan_safety` is a separate node on `/cmd_vel` and still stops the robot. It
only stops the follower steering around obstacles.

Structural parameters (`route_path`, frames, `control_period_s`) are not
live: changing them would need the path or the timers rebuilt.

## Tests

```bash
cd src/outdoor_patrol_route && PYTHONPATH=. python3 -m pytest test/ -q
```

Covers the schema round-trip and its rejection paths, and the path geometry
against a closed-form circle. The end-to-end behaviour is validated in
simulation by
[`run_validation.sh`](../outdoor_patrol_sim/scripts/run_validation.sh).

## Watching a run

The validation harness is headless by default, because rendering competes with
physics and the numbers should come from a run that was not being drawn. To
watch one:

```bash
GUI=rviz ./src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/val r4  # lightest
GUI=gz   ./src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/val r4
GUI=1    ./src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/val r4  # both
```

`GUI=rviz` is usually what you want: it is far lighter than Gazebo's renderer
and it is the view that shows the corridor. Needs `$DISPLAY` or a Wayland
socket; the harness checks and refuses early if neither is set.

The RViz preset is [config/route.rviz](config/route.rviz):

| What you see | Meaning |
|---|---|
| green line | recorded centerline being followed |
| white lines | lane edges |
| orange lines | corridor edges — the follower will not steer outside these |
| orange sphere | the live look-ahead point |
| green arrows | Gazebo ground truth (sim only) |
| orange arrows | global EKF estimate; the gap to green is localization error |
| yellow points | `/scan` — the barrier returns that trigger the retreat |

Watch the orange sphere swing right as a barrier comes up. That displacement
*is* the retreat — there is no second control mode to see.

Treat a GUI run as a look, not as the measurement: real-time factor drops to
roughly 0.4, so R4 takes about seven minutes rather than three. The harness
scales its own timeouts to match.

## Field sites

[config/route_alley.yaml](config/route_alley.yaml) is a narrow-corridor
profile: a 4 m alley, walls instead of shoulders, 1.2 m maximum retreat. The
defaults in `route.yaml` assume the 6 m sim road and would steer 0.70 m into a
wall there. Derive your own before driving a new site:

```
corridor_half_width_m  <=  (width / 2) - robot_half_width - 0.45
trigger_range_m        >=  |max offset| / ramp_lateral_per_m + 3   # vehicle lag
```

The follower prints what it derived at start-up; read that line every run.

Outdoors there is no ground truth, so the base_link differential test compares
the two recordings against each other instead:

```bash
score_route.py --compare alley_corrected.yaml alley_control.yaml
```

The uncorrected track should sit ~0.42 m to the *right* of the corrected one.
The sign is the point: a separation of the right size but the wrong sign means
the lever arm is being added rather than subtracted.

Step-by-step field procedure:
[doc/eng/plans/field-validation-alley.md](../../doc/eng/plans/field-validation-alley.md).
