# Implementation plan — migrate patrol navigation to Nav2

> Imported unchanged except for these link paths and this note. Progress,
> per-phase implementation plans and validation instructions live beside it:
> [progress.md](./progress.md).

- **Supersedes:** the custom `route_follower` in
  [`outdoor_patrol_route`](../../../../src/outdoor_patrol_route) as the thing
  that drives the robot. The recorder, the route file, the localization stack,
  the safety brake and the sim harness all stay.
- **Status:** proposed. Phases are ordered so that each one ends with a
  scored sim run and a field gate before the next starts.
- **Date:** 2026-09-05
- **Decision record:** to be written as
  `doc/eng/decisions/0004-nav2-vs-custom-route-follower.md` once Phase 1
  parity numbers exist (see [Why migrate](#why-migrate)).

## Why migrate

The custom follower is validated
([issue-8-teach-and-repeat.md](../issue-8-teach-and-repeat.md)) and does lane
changes around partial obstacles. What it cannot do, and what the patrol
mission now needs:

| Need | Custom follower | Nav2 |
|---|---|---|
| Go around partial obstacles inside the corridor | yes, fixed-side lateral offset | MPPI deviates freely inside a keepout mask |
| Retreat to a **named safe spot** on a full block | no — `blocked` means stop in place | `NavigateToPose` + a recovery branch in the BT |
| Safe spots **off** the corridor (pull-out, driveway) | not expressible: `(s, d)` only | any `map` pose inside the mask |
| Reverse motion | none | Hybrid-A* + MPPI both plan reverse |
| Moving obstacles | none | obstacle layer clearing + a BT condition to preempt |
| Rotate-in-place, spiral-out avoidance, recoveries | none | stock behaviours |

The cost is real: costmaps, a behaviour tree, ~40 parameters that interact,
and a controller that needs 20+ Hz of CPU on the RK3588. Phase 1 exists to
measure that cost against the follower's numbers before anything new is built
on top.

**Two design rules carry over unchanged.** Every criterion is scored against
Gazebo ground truth or the world's own geometry, never against Nav2's opinion
of where it is. And nothing is raised to make a run pass:
`corridor_half_width_m`, `sigma_stop_m` and the firmware speed clamps are
walls, not knobs.

## What stays, what changes

```
  KEEP    /odometry/global  ◄── dual EKF + navsat_transform + confidence_gate
  KEEP    route.yaml        ◄── route_recorder (geodetic, base_link, datum)
  NEW     route_to_map      ──► corridor keepout mask + speed mask (+ safe-spot islands)
  NEW     patrol_mission    ──► NavigateThroughPoses goals from the route, status JSON
  NEW     outdoor_patrol_nav──► nav2 params, launch, BT XML, custom BT nodes
  KEEP    scan_safety       ◄── still between /cmd_vel_raw and /cmd_vel (ADR-013)
  RETIRE  route_follower    ──► behind a launch flag until Phase 3 field gate, then removed
```

`route_to_map` is the occupancy grid that issue #8 review question 4
deferred. It is no longer optional: the corridor is enforced by a
`KeepoutFilter` in both costmaps, so every planner and controller is
physically constrained to the road width without controller-specific logic.

### Nav2 component choices

| Component | Choice | Why |
|---|---|---|
| Global costmap | rolling window, no static layer, obstacle layer from `/scan`, keepout filter | there is no occupancy map of the site; GNSS is the map |
| Local costmap | 6 × 6 m rolling, obstacle layer, inflation, keepout filter | inflation radius sized to `clearance_half_width_m` (0.55) |
| Planner | Smac Hybrid-A*, `DIFF` model, `minimum_turning_radius: 1.5` | matches the 1.5 m chassis minimum in issue #8; reverse allowed |
| Controller | MPPI, `DiffDrive` model, 20 Hz; RPP registered as `FollowPathRPP` fallback | MPPI is the reason to migrate; RPP is the exit if RK3588 cannot hold 20 Hz |
| Goal source | `NavigateThroughPoses` over stations subsampled every 10 m | dense enough that Hybrid-A* returns the centerline on a clear lane |
| Velocity smoother | `max_velocity: [1.0, 0, 0.67]` | mirrors `chassis.yaml` / firmware clamps |
| Safety | `scan_safety` kept; Nav2 `collision_monitor` added in Phase 6, not before | independent layer beneath Nav2 exactly as it sits beneath the follower today |

### Status topic and scoring

`patrol_mission` publishes `/patrol_mission/status` as JSON with the same
fields the harness already parses from `/route_follower/status` — `state`,
`station`, `cross_track`, `offset`, `speed`, `sigma_h` — computed from
`/odometry/global` against the loaded centerline, plus `bt_state`,
`safe_spot` and `retreat_attempt`. `score_run.py` gains a `--status-topic`
flag and nothing else, so R3/R4 stay comparable across the migration.

---

## Phases

### Phase 0 — Freeze the baseline (sim + field prerequisites)

Sim: re-run `run_validation.sh teach r3 r4 r5` once on the current follower
and commit the numbers to `runs/baseline/`. They are the parity bar for
Phase 1, not the Results table in the issue-8 plan (that table is one lap
each; take three).

Field: [`field-validation-alley.md`](../field-validation-alley.md) Phases 1
and 2 only — GNSS soak and `yaw_offset`. Both are localization facts Nav2
depends on and neither has a field result yet. Phases 3–6 of that plan are
**not** required; they test the follower being replaced.

> Gate: three baseline runs archived; heading within 10° of true; soak
> σ ≤ 0.05 m for 10 min.

### Phase 1 — Nav2 bring-up and clean-lap parity

Build:

- `outdoor_patrol_nav` package: `nav2.launch.py` (planner, controller,
  bt_navigator, behaviors, velocity_smoother, lifecycle manager),
  `config/nav2_params.yaml`, `bt/patrol.xml` (stock
  `NavigateThroughPoses` tree for now).
- `patrol_mission` node: loads `route.yaml` through `fromLL` like the
  follower does, subsamples stations, sends one `NavigateThroughPoses` goal
  per lap, publishes status.
- Sim: `sim.launch.py nav:=true` swaps the follower for the Nav2 stack.
  `run_validation.sh` gains `NAV=1`.

Controller starts as MPPI. If R3-N cannot hold `controller_frequency` on the
dev box, that is a sim-host problem; the RK3588 question is Phase 3.

> **R3-N pass:** same criteria as R3 — RMS cross-track < 0.25 m, peak
> < 0.5 m, one lap, longest stop < 3 s — **and** within 2× of the frozen
> baseline RMS (0.064 m). If Nav2 tracks a clear road at 0.20 m RMS it
> passes the letter of R3 and fails the point of migrating.
>
> **R5-N pass:** `patrol_mission` reads raw `/um982_driver/fix` through
> `route_file.horizontal_sigma()` and cancels the goal above `sigma_stop_m`;
> ≥ 20 sustained degraded cycles, zero commanded speed. Nav2 does not know
> about GNSS quality; this stays a mission-level rule.

Field: none. This phase does not touch the robot.

### Phase 2 — Corridor mask and partial obstacles

Build:

- `route_to_map`: centerline ± `corridor_half_width_m` → PGM/YAML keepout
  mask at 0.05 m; lane ± `lane_half_width_m` → speed mask. Emits `map`-frame
  files from the route datum so they overlay the sim world exactly (Phase 0
  proved the pinned datum lands within 18 mm). `--check` mode like
  `gen_patrol_road.py`.
- `KeepoutFilter` + `SpeedFilter` in both costmaps.
- MPPI critic weights tuned so it uses the shoulder: `PathAlignCritic` and
  `PathFollowCritic` low enough to leave the lane, `CostCritic` /
  `ObstaclesCritic` high. Record the final weights with the R4-N numbers,
  because they are the migration's real tuning artefact.

> **R4-N pass:** body clearance > 0.15 m at every barrier; `|d| ≤ 3.0 m`
> (never outside the mask — checked against ground truth, not the costmap);
> back to `|d| < 0.2 m` within 10 m of clearing; lap completes; longest stop
> < 3 s. Retreat side is **not** asserted — MPPI picks it — but the
> obstacle world makes a left pass geometrically impossible, so a clearance
> pass implies a right pass.
>
> **R4-N corner:** report settled cross-track through barrier 3 separately.
> The follower's known weak spot was 0.33–0.90 m here; Nav2 should be
> better. If it is not, that is a finding, not a fail.

Field: **alley Phases 5 and 6 with the Nav2 stack**, `route_alley.yaml`
corridor 1.8 m, 2.4 m soft obstacle against the left wall. Same gates:
never touches, back to lane within 10 m, never stationary > 3 s.

> Gate: sim R4-N and alley Phase 6 both pass on the same commit.

### Phase 3 — Compute budget on the RK3588

Not a feature phase; a go/no-go for MPPI. The `deploy/Dockerfile` image
gains `ros-jazzy-nav2-*`; the compose file gets a `nav` profile.

Measure on the robot, stack fully up, during an alley lap:
`controller_server` achieved rate, per-core load, `/cmd_vel` jitter.

> Pass: `controller_frequency` ≥ 20 Hz achieved ≥ 95 % of cycles, total
> load leaves ≥ 1 core free for the perception added in Phase 5.
>
> Fail: switch `FollowPath` to the registered RPP config, drop the global
> planner rate to 1 Hz replanning inside the mask so the *planner* does the
> going-around, re-run R4-N and alley Phase 6. That configuration is then the
> shipping one, and MPPI waits for the Orange Pi 5 Ultra.

This is also the point the follower is removed from `sim.launch.py` and
`route_follow.launch.py` is deleted, with the ADR written from the Phase 1–3
numbers.

### Phase 4 — Safe spots and retreat on a static full block

Build:

- Route file version 2: `safe_spots: [{name, lat, lon, yaw, hold_s}]`.
  Geodetic like the stations, projected through `fromLL` at load. Spots may
  be off the corridor; `route_to_map` cuts an island for each into the
  keepout mask (spot pose ± 1.0 m, plus a 1.5 m-wide connector to the nearest
  corridor point, so Hybrid-A* can reach it).
- Recorder: `/route_recorder/mark_safe_spot` Trigger service; press it
  while parked at the spot during the teach pass.
- Custom BT nodes in `outdoor_patrol_nav`:
  - `NearestSafeSpot` — nearest by *route arc-length behind the robot*, not
    Euclidean, so it never picks the one across the road.
  - `WaitForCorridorClear` — calls `ComputePathToPose` to the next station
    every 2 s; succeeds when a path exists.
- `bt/patrol.xml`: `RecoveryNode` around the navigate subtree whose recovery
  branch is `NearestSafeSpot → NavigateToPose(spot, reverse allowed) →
  Wait(hold_s) → WaitForCorridorClear → NavigateThroughPoses(remaining)`.
  Stock spin/backup/wait recoveries come **after** it, not before. Triggers:
  `ComputePathToPose` failure inside the mask, or the progress checker
  aborting. `max_retreat_attempts` then mission abort.
- Sim: `gen_patrol_road.py --full-block` emits a barrier spanning the whole
  corridor at s = 46 (mid-corner, the hard one) and two spots: an on-corridor
  pull-out at s = 30, d = −2.4, and an **off-corridor** one at s = 20, 4 m
  right of the centerline.

> **R6 pass:** `bt_state` reaches `retreating` within 10 s of the block;
> the chosen spot is the s = 30 one (nearer along the route); the robot
> reaches it within 0.5 m, holds; the barrier is removed via
> `gz service ... /world/patrol_road/remove` mid-run; `WaitForCorridorClear`
> fires and the lap completes. Never contact, never outside the mask.
>
> **R7 pass:** same, with the s = 30 spot removed from the file so the
> off-corridor spot is chosen, proving the mask island and connector work.
> Then a second barrier dropped **behind** the robot during retreat: no
> contact, and either the next spot or a clean abort.

Field: alley, second box closing the 1.6 m gap. Spot = the pull-out at the
alley entrance. Gate: reverses to it, holds, resumes when the box is
removed, `retreat_attempt` reads 1.

### Phase 5 — Moving obstacles, LiDAR only

Build:

- `scan_tracker` in `outdoor_patrol_safety`: cluster `/scan`, Kalman-track
  centroids in `map`, publish tracks with velocity. The RPLIDAR C1's ~12 m
  range gives ~1.5 s of lead at 30 km/h, which is the number this phase
  will confirm or refute.
- BT condition `TrafficApproaching`: a track closing along the corridor
  with `time_to_contact < retreat_time_needed`, measured in Phase 4 as the
  time from trigger to arrival at the spot. Preempts the navigate subtree
  and enters the Phase 4 retreat branch early.
- Obstacle layer `observation_persistence` and raytrace clearing tuned so a
  passed mover does not leave a ghost the planner treats as a block.

> **R8 pass:** Gazebo actor driving the centerline toward the robot at 2,
> 4 and 8 m/s. At 2 and 4 m/s the robot is at a spot before the actor
> reaches its position. 8 m/s is **reported**: the expectation is that it
> fails on lidar range alone, and the margin by which it fails sizes the
> camera requirement in Phase 6.

Field: person jogging toward the robot, then a bicycle. No cars.

### Phase 6 — Camera vehicle detection

The RKNN YOLO pipeline on the Orange Pi publishes detections into the same
track topic as `scan_tracker`, so nothing above it changes. Range 50–100 m on
a car, 6–12 s of lead. Nav2 `collision_monitor` is added here as a second
brake now that there are two obstacle sources.

> Field: own car on the closed road at 10 km/h, then 20. Escalate only after
> false-positive rate on parked cars over five laps is measured and below
> one spurious retreat per lap.

### Phase 7 — Hardening

Rosbag on every retreat (`deploy/data/events/`), soak N laps in sim with
randomized block and spot placement, a wiki page for the first field retreat
that goes wrong, and the ADR.

---

## Risks carried into this plan

- **MPPI on the RK3588.** Phase 3 exists because of it. The RPP fallback is
  designed in from Phase 1 so the answer is a config change, not a rewrite.
- **Costmap ghosts from a low lidar.** The C1 sits 0.20 m above ground. Long
  grass and kerbs will enter the obstacle layer where the follower's sector
  check ignored them. Expect Phase 2 field tuning of `obstacle_max_range`,
  `min_obstacle_height` and raytrace clearing.
- **Keepout mask vs. real road edge.** The mask is only as good as the
  teach-pass centerline plus a global half-width. The multi-pass shoulder
  measurement from issue #8 review question 2 becomes more important, not
  less, because Nav2 will actually drive to the mask edge.
- **BT preemption while reversing.** A `TrafficApproaching` tick during a
  Phase 4 retreat must not restart the retreat from scratch. R7's
  block-behind case is the sim proxy for this.
- **Two obstacle sources disagreeing** in Phase 6. The track topic needs a
  source field and the BT condition needs a rule for lidar-only vs
  camera-only detections before the first car test.
