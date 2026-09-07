# Nav2 migration — progress

Tracker for [plan.md](./plan.md). One row per phase; a phase is **done** only
when its scored sim run passes and the numbers are pasted in below.

**Last updated:** 2026-09-06

## Status

| Phase | What it delivers | State |
|---|---|---|
| 0 — freeze the baseline | three scored runs of the current follower | **done** — 3/3 PASS, mean R3 RMS 0.0645 m, `NAV_MAX_RMS` = 0.129. [baseline.md](./runs/baseline/baseline.md), [phase-0.md](./phase-0.md) |
| 1 — Nav2 bring-up + clean-lap parity | `outdoor_patrol_nav`, R3-N, R5-N | **done** — R3-N and R5-N both PASS. Numbers below. [phase-1.md](./phase-1.md), [validation](./phase-1-validation.md) |
| 2 — corridor as a costmap filter | `route_to_map`, keepout + speed masks, R4-N | not started |
| 3 — RK3588 CPU budget + ADR-0004 | MPPI vs RPP measured on target | not started |
| 4 — safe-spot retreat | route schema v2, BT nodes, R6-N | not started |
| 5 — moving obstacles | `scan_tracker`, BT preemption | not started |
| 6 — `collision_monitor` | last-resort stop independent of Nav2 | not started |

## Phase 1 results

Measured 2026-09-06 in the ROS 2 Jazzy devcontainer, headless, on the machine
described in [baseline.md](./runs/baseline/baseline.md). Artefacts in
[runs/phase-1/](./runs/phase-1/).

**R3-N** — clean lap under Nav2. All five gates hold:

| Field | Gate | Measured |
|---|---|---|
| `cross_track_rms_m` | < 0.129 (2× baseline) | **0.0883** |
| `cross_track_max_m` | < 0.50 m | **0.2192** |
| `laps` | ≥ 0.98 | **0.9880** |
| `longest_stop_s` | < 3.0 s | **1.68** |
| `lateral_min/max_m` | inside ±3.0 m corridor | **−0.075 / +0.219** |

**R5-N** — degraded fix under Nav2. Both gates hold:

| Field | Gate | Measured |
|---|---|---|
| `degraded_cycles` | ≥ 20 | **452** |
| `final_speed_ms` | ≤ 0.001 | **0.0** |

Nav2 tracks the clean road at 0.088 m RMS against the follower's 0.0645 m —
1.37× the baseline, inside the 2× bar. That is the number the ADR-0004
decision record should quote: the migration costs about a third more
cross-track error on the easy case, in exchange for the capabilities in the
plan's [Why migrate](./plan.md#why-migrate) table.

## Findings

Five things found by reading the existing code before anything was run, then
three more found by running it. Recorded here so a later phase does not start
by trusting the plan over the code.

### 1. `d_cmd` is mandatory in every status message

`score_run.py:200` reads `s['d_cmd']` with an unguarded dict access. The plan
document's prose names the status fields `station` and `offset`; the code reads
`s`, `d_cmd` and `state`. `patrol_mission` therefore mirrors the follower's
exact key names and always emits `d_cmd: 0.0`.

Consequence, and it is a good one: with `d_cmd ≡ 0`, `cross_track` collapses to
`lateral_active` — the true lateral position measured against the world's own
centerline. That is what the metric should always have been. It also means
`--status-topic` is the only scorer change Phase 1 needs.

### 2. `--status-topic` is not enough for Phase 2

Under `--expect-obstacles`, `check()` asserts:

- `d_min_m <= -min_retreat_m` — "never retreated" (`score_run.py:346`)
- `d_max_m <= 0.05` — "retreated to the LEFT" (`score_run.py:350`)

Both read the **commanded** offset, which is identically zero under Nav2, so
both misfire: the first always fails, the second always passes. Phase 2 must
replace them with a ground-truth lateral-excursion measure and drop the
retreat-side assertion entirely — the plan document itself says MPPI picks the
side. Until that lands there is deliberately **no `r4n` case** in
`run_validation.sh`.

### 3. `NavigateThroughPoses`, not `FollowGPSWaypoints`

Nav2 ships `nav2_waypoint_follower` with a `FollowGPSWaypoints` action that
would do the geodetic projection for us, which looks like the more standard
choice. It is the wrong one: it runs one `NavigateToPose` per waypoint and
comes to a **stop** at each. With stations every 10 m that is a stop every
10 m, failing R3-N's "longest stop < 3 s" by construction.
`NavigateThroughPoses` treats intermediate poses as via-points. The cost is
making the `/fromLL` calls ourselves — about 15 lines.

### 4. `motion_model_for_search: DIFF` in the plan document is invalid

`SmacPlannerHybrid` accepts only `DUBIN` and `REEDS_SHEPP`. `DIFF` belongs to
`SmacPlanner2D` and the lattice planner, and would be rejected at configure
time — taking the **whole** parameter set with it, which reads as "Nav2 will
not start" rather than "one parameter is wrong". `config/nav2_params.yaml` uses
`REEDS_SHEPP`, which is also the model that plans the reverse motion Phase 4
needs for the retreat.

### 5. `robot_radius: 0.3024` understates the robot

0.3024 m is the wheel half-width `score_run.py` uses for clearance, and it is
correct for that. As a costmap footprint it is not: the body box reaches 0.56 m
forward of `base_link`, so a circle of that radius lets the nose clip an
obstacle the costmap called clear. Both costmaps use an explicit polygon
instead:

```yaml
footprint: "[[0.56, 0.31], [0.56, -0.31], [-0.12, -0.31], [-0.12, 0.31]]"
```

### 6. A loop route sent as one `NavigateThroughPoses` goal is already complete

Found by running it. `NavigateThroughPoses` goal-checks only the **final**
pose; the intermediate ones are via-points. A patrol route is a closed loop,
so its last station is its first — the robot is standing on the final goal
pose when the goal is sent. `bt_navigator` logged

```
Begin navigating from current location through 10 poses to (-8.58, -13.57)
Reached the goal!            <- 19 ms later
Goal succeeded
```

against a start pose of `(-8.573, -13.573)`. The first R3-N scored
`travelled 0.0 m (0.00 laps)` and, because the route "completed", it did so
without any error anywhere in the stack.

`patrol_mission` now splits the lap into chunks of at most `max_goal_span_m`
(50 m) and sends the next chunk from the result callback, so no goal ever ends
where the robot currently is. The span is a distance, not a pose count, so
changing `station_spacing_m` does not silently change how many chunk
boundaries there are — each boundary is a brief stop that R3-N's
`longest_stop_s` gate has to absorb. One boundary per 100 m lap cost 1.68 s
against the 3 s limit.

Phase 4 should re-check this when the retreat branch lands: a `NavigateToPose`
retreat to a safe spot the robot is already parked on has the same failure
mode, and the same silence.

### 7. `station_spacing_m: 10.0` cannot represent a 5 m corner

Also found by running it, and the reason the second R3-N scored 0.608 m RMS
with a 2.45 m peak excursion — an order of magnitude off the 0.129 m bar.

The plan document justifies 10 m as "dense enough that Hybrid-A* returns the
centerline on a clear lane". The sim road is a rounded square with
`corner_radius_m: 5.0`, and a chord of spacing `s` across an arc of radius `R`
departs that arc by `R·(1 − cos(s/2R))`:

| `station_spacing_m` | departure from the arc |
|---|---|
| 10.0 | 2.30 m |
| 3.0 | 0.223 m |
| **2.0** | **0.0997 m** |
| 1.0 | 0.025 m |

2.30 m predicted against 2.45 m measured, so the geometry accounts for
essentially all of the error. The via-poses do carry the recorded heading, so
this is not a missing-orientation bug: Smac corners at the planner's
`minimum_turning_radius` of 1.5 m, not at the road's 5 m, and heading
constraints at the endpoints do not hold the path onto the arc between them.

`station_spacing_m` is now **2.0**, the largest spacing whose geometric term
(0.0997 m) sits under the parity bar on its own while staying above the 1.5 m
turning radius, so consecutive stations remain connectable without a
Reeds-Shepp loop. This is a property of the corner radius rather than a knob:
re-derive it if `corner_radius_m` changes, and note that a real site with
tighter corners needs a smaller number.

### 8. `action_server_is_ready()` cannot see a lifecycle node's activation

A Nav2 lifecycle server creates its action server in `configure()` and rejects
every goal until `activate()`. `rclcpp_action`'s `action_server_is_ready()`
reports only that the server has been **discovered**, so it goes true at
configure time and cannot distinguish the two states. For `bt_navigator` the
gap was 1.70 s, and the first goal landed inside it:

```
21:41.98  Creating navigator id navigate_through_poses
21:42.12  Action server is inactive. Rejecting the goal.
21:43.82  Activating
```

The original code treated sending as success — it cancelled the retry timer
before the goal outcome was known — so one rejected goal stalled the mission
permanently, and the harness sat in `R3-N -- following the route` until it
timed out. Acceptance is now proven by `goal_response_callback`, which cancels
the timer only when a handle comes back and re-arms it on rejection. In
practice two retries are needed, one second apart.

The flags that coordinate this (`goal_active_`, `goal_pending_`, `started_`,
`finished_`) are `std::atomic<bool>`: the goal response arrives on the action
client's callback group while the retry timer runs on another, and both are
live on the `MultiThreadedExecutor`.

### 9. The teach driver's `lookahead_m` has the same corner-radius bug

Found by building an 18 ft driveway world
([runs/driveway/](./runs/driveway/)) and driving it. The first Nav2 lap there
scored 0.145 m RMS — and the *recorded route* scored 0.147 m against ground
truth. Nav2 was adding nothing; it was accurately following a bad route.

`sim_route_driver`'s pure pursuit cuts a corner of radius `R` by about
`L²/(8R)` for lookahead `L`. The default `L` is 1.5 m:

| | L | R | predicted | measured |
|---|---|---|---|---|
| 100 m road | 1.5 | 5.0 | 0.056 m | 0.028–0.034 m |
| driveway | 1.5 | 1.0 | 0.281 m | 0.271 m |
| driveway, fixed | 0.45 | 1.0 | 0.025 m | 0.031 m |

Re-teaching at 0.15 m/s rather than 0.35 changed nothing (0.147 → 0.146 m),
confirming it is geometric, not dynamic.

This is finding 7 again — a length that is correct at one corner radius and
quietly wrong at another — in a different component, and it bounds every R1
and R3 number in this repository: the 100 m road's own teach pass carries a
0.056 m predicted corner cut against a 0.0645 m baseline RMS. **A meaningful
share of the "baseline" may be the teach driver, not the follower.** Worth
re-running Phase 0 with `TEACH_LOOKAHEAD=0.5` before ADR-0004 quotes the
0.0645 m figure as the follower's accuracy.

`run_validation.sh` now exposes `TEACH_LOOKAHEAD`, defaulting to the unchanged
1.5 m so no existing number moves.

### 10. `minimum_turning_radius: 1.5` is not a chassis limit

`config/nav2_params.yaml` sources it as "the 1.5 m chassis minimum from issue
#8". Issue-8 states no chassis turning limit anywhere; its only 5 m radius is
the *world's* corner radius. `chassis.yaml` is a 0.545 m-track differential
drive with a caster, which pivots in place, so the true minimum is ~0.

The driveway config uses 0.4 m and Smac plans 1.0 m corners without
complaint. On the 100 m road 1.5 m is larger than the road requires and is
part of why Smac corners tighter than the arc (finding 7). Reducing it there
is untested and should be measured, not assumed — but the justification
currently in the comment is not correct.

## Open risks

Ranked. Risks 2–4 below were open before Phase 1 ran; what the runs settled is
noted against each.

1. **MPPI CPU on the RK3588.** Still measured nowhere. Phase 3 exists for
   this; `FollowPathRPP` is already configured so the fallback is a one-line
   change to `bt/patrol.xml`. Note the dev box held `controller_frequency`
   without complaint, which says nothing about the RK3588.
2. **`longest_stop_s` at the end of a run.** ~~Predicted ~1.5 s against a 3 s
   limit.~~ Measured 1.68 s in R3-N and 1.54 s in R5-N, so the prediction was
   right and the margin is real but not large. Phase 1 adds one mid-lap chunk
   boundary (finding 6) which is inside that same number — if a future phase
   raises `max_goal_span_m` boundaries per lap, this is the gate that will
   notice first.
3. **Goal tolerance vs station spacing.** ~~If the robot circles a station
   instead of passing it, `RemovePassedGoals radius="0.7"` is the knob.~~ Did
   not occur, at either 10 m or 2 m spacing. At 2.0 m spacing the stations are
   still ~3× the removal radius apart, so the pairing is comfortable; below
   ~1.4 m spacing it would stop being so.
4. **`enable_stamped_cmd_vel`.** ~~If the robot never moves, check the
   `/cmd_vel_raw` message type first.~~ Settled: the robot drives, so the
   whole velocity chain — `velocity_smoother` → `scan_safety` → the sim — is
   type-correct as configured.
5. **`station_spacing_m` is tuned to the sim road's 5 m corners.** New, and
   the one to carry into the field. 2.0 m is derived from
   `corner_radius_m: 5.0` (finding 7). A real site with tighter corners needs
   a smaller value, and nothing in the code checks this: the failure mode is a
   quiet 2 m corner-cut, not an error. Phase 2's keepout mask turns it into a
   hard constraint, which is the proper fix.
6. **The Phase 0 baseline may be partly the teach driver.** Finding 9: the
   teach pass's 1.5 m lookahead predicts a 0.056 m corner cut on the 100 m
   road, against a baseline R3 RMS of 0.0645 m. If a re-teach at
   `TEACH_LOOKAHEAD=0.5` moves the baseline, then `NAV_MAX_RMS` — and the
   1.37× parity ratio — are measuring the harness as much as the follower.
   Cheap to check: three `teach r3` runs.
7. **Both R3 gates are written for a 100 m lap.** The driveway run
   ([runs/driveway/](./runs/driveway/)) scored 0.057 m RMS — better than the
   100 m road — and still failed on `laps` (0.97 vs 0.98, where 0.98 of a
   17.7 m loop leaves only 0.35 m of slack) and `longest_stop_s` (4.0 s, one
   replan on a 186 s run). Any future world of a different size needs the
   gates restated as fractions of *its* geometry, not the numbers inherited
   from the 100 m road.

## Log

- **2026-09-05** — Phase 0 and Phase 1 written. Package `outdoor_patrol_nav`,
  docs under this directory, `--status-topic` added to `score_run.py`, `nav:=`
  added to `sim.launch.py`, `r3n`/`r5n` added to `run_validation.sh`. Nothing
  built or run. Next: [phase-0-validation.md](./phase-0-validation.md).
- **2026-09-06** — Phases 0 and 1 built, run and scored in a ROS 2 Jazzy
  devcontainer. Nav2 was not installed in that container and `rosdep install`
  was silently installing nothing (see below), so the first job was making the
  environment reproducible.

  Phase 0: 3/3 PASS, R3 RMS 0.0612 / 0.0642 / 0.0681 m, mean 0.0645 →
  `NAV_MAX_RMS` 0.129, now the default in `run_validation.sh`.

  Phase 1: four code defects, none of which could have been found without
  running. In the order they surfaced —
  1. `goal_sent_` was never declared: the package had never compiled.
  2. `static_cast<long>` tripped `cpplint runtime/int`.
  3. The goal-acceptance race, finding 8.
  4. The closed-loop goal, finding 6.

  Then one parameter that was wrong on the evidence rather than broken:
  `station_spacing_m` 10.0 → 2.0, finding 7. R3-N 0.0883 m RMS and R5-N
  452 degraded cycles, both PASS.

  Toolchain: `scripts/rosdep-install.sh` added and wired into
  `devcontainer.json`, `setup.sh` and both stages of `deploy/Dockerfile`.
  `outdoor_patrol_bringup` exec_depends on `sllidar_ros2`, a submodule that is
  absent unless `./setup.sh` has been run with git SSH credentials, and rosdep
  treats one unresolvable key as fatal for the entire invocation — so it
  installed **none** of the other ~20 keys and the visible symptom was "nav2
  is missing". The wrapper skips keys provided from `src/`.

  Next: Phase 2, starting with the R4 scorer rewrite in finding 2.
