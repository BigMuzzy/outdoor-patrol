# outdoor_patrol_validation

One screen for running the field validation procedure in
[`doc/eng/plans/field-validation-alley.md`](../../doc/eng/plans/field-validation-alley.md).

RViz on the left, every gate on the right, nothing to check in a second
terminal.

![layout](config/field_validation.rviz)

```bash
ros2 launch outdoor_patrol_validation field_dashboard.launch.py
```

## Why

The plan is good and the gates in it are precise. The problem is reading them.
Most are *temporal* — "σ ≤ 0.05 m **held for 10 minutes** with no dropouts",
"`d_cmd` **never** goes positive", "never stationary more than 3 s" — and none
of those can be read off a scrolling `ros2 topic echo`. They have to be
accumulated by something watching continuously.

Doing that by eye, outdoors, beside a moving robot, with a laptop in one hand
and a kill switch in the other, is how a failed phase gets written down as a
pass. The plan says it itself: *a phase that passed and you do not know why is
not a result.*

So this package watches instead. It knows the same thresholds the stack is
configured with, it latches violations, and it writes the report at the end.

## The two halves

| | What it is | Where it runs |
|---|---|---|
| `field_dashboard` | Python node. Subscribes to the stack, evaluates the Phase 0–7 gates, publishes one JSON state document, writes the report. | Dev box **or** robot. No display needed. |
| `FieldDashboardPanel` | C++ RViz2 panel. Renders that document, sends button presses back. | Dev box, inside RViz. |

The split is the point. All the judgement is in the node, where it is
[unit-tested](test/test_phases.py) and where it keeps running if RViz is closed
or the laptop lid goes down. The panel holds no thresholds and decides nothing.

For a long soak, run the node on the robot as well and take its report off with
the rest of the run data:

```bash
ssh robot 'ros2 run outdoor_patrol_validation field_dashboard \
    --ros-args --params-file /data/field_dashboard.yaml'
```

## Using it

Select a phase in the list, do what the blue box tells you, press **Start**.
The gate table below updates live; the phase goes green on its own when every
gate is satisfied.

* **Start / Stop** — Start resets the phase and begins accumulating. There is
  no "pause": a phase is one continuous attempt, because most of these gates
  are about continuity.
* **Mark pass / Mark fail** — an operator verdict that overrides the computed
  one, for the gates no sensor here can settle: Phase 0's "container `Up`, no
  restart loop", or Phase 4 scored the plan's way with `score_route.py
  --compare`. Marked phases are labelled `(manual)` in the report.
* **Reset** — clears a latched failure so you can re-run.
* **Write report** — markdown into `report_dir` (default `runs/field/`). Also
  written automatically on shutdown, so a crashed laptop does not cost you the
  day's results.

### The tiles

Colour is the judgement, not decoration: green in spec, amber marginal, red
out of spec, **grey means stale** — the topic stopped publishing. A dead
sensor never renders as a plausible number, which is the one thing a terminal
full of `echo` output cannot promise you.

### What the gates actually check

| Phase | Automatic | Needs you |
|---|---|---|
| 0 Bring-up | every required topic live, `map`→`base_link` resolving, held 10 s | the docker side (`Up`, no restart loop) — mark it |
| 1 GNSS soak | worst σ, 10 min continuous hold, dropout count, correction age | park the robot mid-alley |
| 2 Heading | reported yaw vs. the course actually driven, over 2 m | drive the 2 m |
| 3 Teach pass | predicted sample count, route length, loop closure, worst fix | run the recorders, drive the alley |
| 4 Lever arm | antenna offset in body axes via `/fromLL`, **including its sign** | stand still for a few seconds |
| 5 Autonomous run | reached the end, worst cross-track, wall clearance, stayed in lane | walk alongside with the kill switch |
| 6 Obstacle | `d_cmd` never positive, full retreat, resume distance, stall time, clearance | place the barrier |
| 7 GNSS fault | σ rose, speed fell, robot stopped, never fast on a bad fix | block the antenna |

Two are worth calling out.

**Phase 2** automates the half of the heading check that is objective. The plan
asks you to compare the reported yaw against a phone compass and then drive 2 m
to confirm. The dashboard captures your position at Start and, once you have
moved 2 m, compares the course you actually travelled against the heading the
receiver reported — both ENU, so they must agree. If they are 180° apart it
says so, and names the one-line fix.

**Phase 4** does live what the plan does offline with two recorders. It
converts the raw antenna fix into the map frame through `/fromLL` and expresses
the offset from `/odometry/global` in body axes; that offset *is* the lever
arm, and it should reproduce `gnss_link` from `chassis.yaml`. The **sign** is
checked as its own gate, because right magnitude with wrong sign means the
lever arm is being added where it should be subtracted — the failure a
magnitude-only comparison waves through. Run `score_route.py --compare` too and
mark the phase if you want the recorded-route version on the record.

Phase 3 cannot read `route_recorder` — it publishes no status — so it applies
the recorder's own sampling rule to `/odometry/global` and reports what the
file *should* contain. Compare it against the saved YAML. A disagreement means
the recorder read a different source than you think, which is the silent
`--params-file` ordering failure the plan warns about.

## Rehearse it first

A trip to the alley is three hours and the phases are sequential. Several gates
only fire on failures you cannot conjure outdoors — you cannot ask a receiver
to report a 180° heading error. So there is a synthetic stack:

```bash
ros2 launch outdoor_patrol_validation rehearsal.launch.py scenario:=obstacle
```

| Scenario | What it should do |
|---|---|
| `nominal` | Phases 0–5 all green |
| `heading_flip` | Phase 2 FAIL, naming `yaw_offset` |
| `bad_rtk` | Phase 1 never completes its hold |
| `obstacle` | Phase 6 retreat → resume, PASS |
| `wrong_side` | Phase 6 FAIL on `d_cmd never positive` |
| `lever_arm_flipped` | Phase 4 FAIL on the sign check |
| `gnss_fault` | Phase 7 slows, then stops |
| `driveway` | The square circuit below, Phases 0–5 |

It publishes on the **real topic names**, so do not run it while the robot is
up — it would fight the drivers for `/odometry/global`.

## Running it against the Gazebo sim

The rehearsal above is a bench tool. For the real thing —
[`outdoor_patrol_sim`](../outdoor_patrol_sim) with physics, a real lidar and
the actual dual-EKF — three terminals:

```bash
# 1. the simulator. Use the ROAD world for route work: the default
#    patrol_yard.sdf is a small enclosure and you will drive into a wall.
ros2 launch outdoor_patrol_sim sim.launch.py \
    world:=$(ros2 pkg prefix outdoor_patrol_sim)/share/outdoor_patrol_sim/worlds/patrol_road.sdf

# 2. the dashboard + the one-screen RViz layout
ros2 launch outdoor_patrol_validation field_dashboard.launch.py \
    params_file:=$(ros2 pkg prefix outdoor_patrol_validation)/share/outdoor_patrol_validation/config/field_dashboard_sim.yaml \
    site:=gazebo

# 3. teleop. The remap is REQUIRED -- the safety brake sits between
#    /cmd_vel_raw and /cmd_vel, and without it you bypass the brake.
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

Then drive the phases exactly as you would outdoors: select one, press Start,
do the thing. Measured on `patrol_road.sdf`, Phases 0–5 all reach PASS; the
lever arm comes back as **−0.421 / +0.282 m** against a shipped
`gnss_link` of −0.42 / +0.28, which is the sharpest end-to-end check in the
package.

Phases 6 and 7 need stimuli the plain road world does not have — use
`patrol_road_obstacles.sdf` for the barrier, and Phase 7 needs a degraded fix
that `gnss_sim` does not currently model.

### Two things about the sim that are not faults

**The fix quality tile reads `from covariance (no NMEA)`.** `gnss_sim`
synthesises a `NavSatFix` directly, so there is no receiver and no raw
sentences — satellite count, HDOP, correction age and base station ID are all
blank. Nothing gates on them. Phase 3's `worst fix` classifies from the status
and sigma instead, which is what `route_recorder` does anyway.

**The robot stops dead with `obstacle: true`.** That is `scan_safety` doing its
job at 0.5 m, not a bug. It only gates *forward* velocity, so reverse and
turn-in-place still work — back it off and turn away:

```bash
ros2 topic pub --rate 10 /cmd_vel_raw geometry_msgs/Twist '{linear: {x: -0.3}}'
```

Run **one** sim at a time. Two `gz sim` servers share gz-transport topic names
and the second silently steals `/cmd_vel` and `/scan` from the first.

> `gz` lives inside the ROS prefix (`gz_tools_vendor`), not `/usr/bin`. If
> `which gz` comes up empty you have not sourced `/opt/ros/$ROS_DISTRO/setup.bash`.

### The panel runs the procedure

**Phase 3 Start launches a `route_recorder`; Stop saves it.** Phases 5 and 6
Start a `route_follower` on the route you just taught. So the loop is:

| | You do | The dashboard does |
|---|---|---|
| Phase 3 | Start → teleop the route → Stop | records, then **saves** to `route_dir` |
| Phase 5 | drive back to the start → Start | launches the follower on that route |

The panel shows what it has running, under the blue action box:

```
recorder: RUNNING    follower: idle    route: simtest_20260905_053420.yaml
```

Two parameters control it, and they differ by profile on purpose:

| | `manage_recorder` | `manage_follower` |
|---|---|---|
| `field_dashboard_sim.yaml` | true | **true** |
| `field_dashboard.yaml` (alley) | true | **false** |
| `field_dashboard_driveway.yaml` | true | **false** |

Recording is safe — it only reads topics and writes a file, and the
alternative is a teach pass that silently records nothing. **Starting the
follower moves the robot**, so outdoors that stays a deliberate act by whoever
is holding the kill switch: the field profiles leave it off and the panel
tells you the command instead.

`child_use_sim_time` must be **true** against the sim. The sim publishes
`/clock`, and a recorder on wall time disagrees with the EKF about when
everything happened.

If you would rather drive it all by hand, set both to false and launch them
yourself:

```bash
ros2 run outdoor_patrol_route route_recorder --ros-args \
    -p use_sim_time:=true -p output_path:=/tmp/my_route.yaml -p loop:=false
ros2 service call /route_recorder/save std_srvs/srv/Trigger

ros2 launch outdoor_patrol_route route_follow.launch.py \
    route_path:=/tmp/my_route.yaml use_sim_time:=true nominal_speed_ms:=0.5
```

When the follower is not running — whoever was meant to start it — Phases 5
and 6 say so on the first gate row rather than sitting on PENDING:

```
  fail   route_follower publishing   NO PUBLISHER on /route_follower/status
```

Expect `degraded` for a second or two at the start of a run: the follower
refuses to move until it has a fix. And drive the robot back to the *start* of
the route before following it — it will not reverse to get there.

### Live tuning from the panel

The **Follower** box retunes `route_follower` — and it works **before** a run
as well as during one. The corridor width and the avoidance switch are what
you decide while setting a run up, and the follower does not exist yet to
receive them, so the dashboard owns them:

| Box title | Meaning |
|---|---|
| *Follower (applies on next run)* | no follower yet; your settings are held and handed over when Phase 5/6 launches one |
| *Follower (live)* | a follower is running; changes reach it immediately |

| Control | Parameter | When it applies |
|---|---|---|
| **Obstacle avoidance** | `avoidance_enabled` | on toggle |
| **Draw corridor** | `show_corridor` | on toggle — display only |
| **speed** | `nominal_speed_ms` | on **Apply** |
| **corridor** | `corridor_half_width_m` | on **Apply**, offsets recomputed |

**Draw corridor** hides the lane (white) and corridor (orange) bands, keeping
the green route and the look-ahead point. On a tight route the offset curves
fold and the scene fills with crossing lines; this gets them out of the way
without touching what the follower does. Hiding publishes an explicit
`DELETE` for those markers — simply not republishing would leave the last
copy in RViz for ever, so the toggle would look broken.

Starting values come from `follower_avoidance`, `follower_speed_ms` and
`follower_corridor_m` in the profile, so each site opens with sensible numbers
(1.8 m corridor for the alley, 0.55 m for the driveway).

**Turn avoidance off for a GNSS-only run.** The follower then tracks the taught
line and nothing else: the corridor is not consulted, the lidar is not read for
avoidance, and `d_cmd` is held at zero. Cross-track then measures localization
and control alone, with no retreat manoeuvre mixed in to explain away a wander
— which is what you want when the question is "how good is the fix?" rather
than "does avoidance work?".

It does **not** disable the forward safety brake. `scan_safety` sits
downstream on `/cmd_vel` and still stops the robot for anything in front of
it. This only stops the follower steering *around* things.

Values are validated by the follower, and a rejected one is reported rather
than silently dropped:

```
nominal_speed_ms REJECTED: nominal_speed_ms must be positive
```

The same works from a terminal, which is the fallback if RViz will not start:

```bash
ros2 param set /route_follower avoidance_enabled false
ros2 param set /route_follower corridor_half_width_m 1.0
```

Structural parameters — `route_path`, the frames, `control_period_s` — are
deliberately *not* live. Changing them mid-run would need the path or the
timers rebuilt, so they stay in the profile where they are reviewable.

The dashboard also picks up the newest route in `route_dir` at startup, so
closing and reopening the panel does not lose a teach pass.

> **Run one dashboard at a time.** Two of them publishing `~/state` interleave
> their messages, the panel shows alternating data from both, and settings
> appear not to take. Each instance now detects this and says so, but the
> cheap check is `pgrep -af field_dashboard` before you launch.

## Running it on the real robot

**The dashboard node runs ON THE ROBOT; only RViz runs on the dev box.**

That split is what makes teach-and-repeat work with nothing to copy. Phase 3
launches `route_recorder` wherever the node is, so running it on the robot
writes the route straight to `/data/routes` on the robot's own disk — exactly
where `route_follower` reads it from. Run the node on the dev box instead and
the route lands there, invisible to the follower.

It also means a dropped WiFi link costs you the *view*, not the run: the gates
keep accumulating and the report is still written.

**On the robot** — the stack, then the dashboard:

```bash
ssh robot 'cd ~/code/outdoor-patrol && \
  NTRIP_PARAMS=/data/ntrip.yaml docker compose -f deploy/docker-compose.yaml up -d'

# alley (default)
ssh robot 'cd ~/code/outdoor-patrol && \
  docker compose -f deploy/docker-compose.yaml --profile validation up -d'

# driveway
ssh robot 'cd ~/code/outdoor-patrol && \
  SITE=driveway SITE_SUFFIX=_driveway \
  docker compose -f deploy/docker-compose.yaml --profile validation up -d'
```

**On the dev box** — RViz only. Check you can see the robot first:

```bash
ros2 topic list | grep field_dashboard   # if empty, see the DDS note below

rviz2 -d $(ros2 pkg prefix outdoor_patrol_validation)/share/outdoor_patrol_validation/config/field_validation.rviz
```

Teleop, third terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

Routes and reports persist on the robot under `deploy/data/` (the bind mount),
so they survive container restarts. Collect them at the end:

```bash
scp -r robot:~/code/outdoor-patrol/deploy/data/routes ./runs/
scp -r robot:~/code/outdoor-patrol/deploy/data/reports ./runs/
```

### Following the route

`manage_follower` is **false** in both field profiles: launching the follower
moves the robot, and outdoors that stays a deliberate act by whoever is
holding the kill switch. The route is already on the robot, so there is still
nothing to copy:

```bash
ssh robot 'docker exec outdoor-patrol ros2 launch outdoor_patrol_route \
    route_follow.launch.py \
    params_file:=/opt/outdoor-patrol/install/share/outdoor_patrol_route/config/route_alley.yaml \
    route_path:=/data/routes/<the file Phase 3 wrote> \
    nominal_speed_ms:=0.4'
```

If you would rather Phase 5 launch it for you — one button, no ssh — set
`manage_follower: true` in the profile. Understand what that changes: pressing
Start in RViz will set the robot moving.

### The robot image does not build the panel

`CMakeLists.txt` builds the RViz panel only when `rviz_common` and Qt5 are
present. The Pi image has no business carrying Ogre and Qt for a plugin it
will never load, so the robot build produces the node alone — and the same
package still builds the panel on the dev box.

### What differs from the sim

| | sim | robot |
|---|---|---|
| dashboard runs on | dev box | **the robot** |
| `route_dir` | `runs/routes` | **`/data/routes`** |
| `child_use_sim_time` | true | **false** |
| `manage_follower` | true | **false** — you launch it |
| `soak_hold_s` | 60 s | **600 s**, the real gate |
| `route_params_file` | `route.yaml` | `route_alley.yaml` / `route_driveway.yaml` |

### If the dev box cannot see the robot

Both ends must be on Cyclone DDS — Fast DDS does not converge on a dual-NIC
dev box bridged to WiFi. Already configured on both sides; verify:

```bash
echo "$RMW_IMPLEMENTATION"                                   # rmw_cyclonedds_cpp
ssh robot 'docker exec outdoor-patrol env | grep RMW_'       # same
ros2 daemon stop && ros2 daemon start
```

See
[dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md](../../doc/eng/wiki/networking/dds-discovery-fails-on-wifi-with-multi-nic-dev-box.md).

## Before the alley: the driveway square

An hour on a driveway buys most of the alley's risk down, and one thing the
alley cannot give you at all.

Teach a **3.5 m square with 1 m rounded corners** on an 18 × 18 ft slab, drive
it as a closed circuit, and you exercise:

* **All four cardinal headings.** A straight teach pass can hide a `yaw_offset`
  that is wrong by 90°; a square cannot, because the repeat comes out visibly
  rotated. This is the single best reason to do it.
* **Corners.** The spline, the recorder's yaw trigger and pure pursuit all
  behave differently on a 1 m fillet than on a straight.
* **Loop closure as a drift measurement.** Drive back to where you started and
  the gap is the localization error over a lap — no ground truth needed, just
  the same patch of ground twice. Phase 5 reports it as *distance from start*.
  Run several laps and watch whether it accumulates.

```bash
ros2 launch outdoor_patrol_validation field_dashboard.launch.py \
    params_file:=$(ros2 pkg prefix outdoor_patrol_validation)/share/outdoor_patrol_validation/config/field_dashboard_driveway.yaml \
    site:=driveway
```

Pair it with
[`route_driveway.yaml`](../outdoor_patrol_route/config/route_driveway.yaml) for
the recorder and follower. Rehearse the whole thing first:

```bash
ros2 launch outdoor_patrol_validation rehearsal.launch.py scenario:=driveway \
    params_file:=$(ros2 pkg prefix outdoor_patrol_validation)/share/outdoor_patrol_validation/config/field_dashboard_driveway.yaml
```

**Do not reuse `route_alley.yaml` here.** Its 1.2 m retreat is wider than a 1 m
corner, so the offset lane inverts and the follower would steer into the
corner. `route_follower` catches it at start-up and says so, but the right
answer is the driveway config, which disables the retreat entirely — an 18 ft
slab has no room for one in either direction. Obstacle avoidance is what the
alley's 4 m width is for.

Phases 6 and 7 need the alley. Everything else transfers.

## Thresholds

In [`config/field_dashboard.yaml`](config/field_dashboard.yaml), and they are
the same numbers the stack itself is configured with:

| Dashboard | Source |
|---|---|
| `sigma_gate_m` | `confidence_gate.yaml` `max_horizontal_sigma_m` |
| `sigma_slow_m`, `sigma_stop_m` | `route_alley.yaml` `route_follower` |
| `lever_arm_x_m`, `lever_arm_y_m` | `chassis.yaml` `gnss_link` xyz |
| `teach_sample_dist_m`, `_yaw_deg` | `route_alley.yaml` `route_recorder` |

Keep them in step —
[`test_thresholds_match_the_shipped_stack_configs`](test/test_phases.py) is the
reminder if one moves. A dashboard that passes a gate the robot fails is worse
than no dashboard.

If your site is not a 4 m alley, re-derive the numbers there first, then mirror
them here. Do not relax one to make a phase go green: as the plan puts it, each
is a wall the robot is not supposed to drive through.

## Interfaces

`~/state` — `std_msgs/String`, JSON, 5 Hz:

```json
{"active": 1, "site": "alley",
 "signals": {"sigma_gated": 0.021, "gga_quality": 4, "...": "..."},
 "phases": [{"index": 0, "name": "Bring-up", "verdict": "pass",
             "checks": [{"label": "...", "value": "...", "status": "pass"}]}],
 "log": ["18:27:44  phase 1 started"]}
```

`~/command` — `std_msgs/String`, JSON:

```json
{"action": "start", "phase": 2}
{"action": "mark", "phase": 4, "verdict": "pass"}
{"action": "report"}
```

JSON over `String` rather than a custom interface package, matching
`route_follower`'s existing `~/status`. It keeps the panel free of generated
headers, and it stays greppable with `ros2 topic echo` when the panel itself is
what is broken:

```bash
ros2 topic echo /field_dashboard/state --field data | head -1
ros2 topic pub --once /field_dashboard/command std_msgs/String \
    '{data: "{\"action\": \"start\", \"phase\": 1}"}'
```

That last pair is the fallback if RViz will not start on site. The procedure
still runs.

## Tests

```bash
colcon test --merge-install --packages-select outdoor_patrol_validation
```

[`test/test_phases.py`](test/test_phases.py) drives every gate with synthetic
snapshots, including the named failure modes — the 180° flip, the reversed
lever arm, and a run that violates cross-track and then recovers, which must
still read FAIL.
