# Phase 1 — Nav2 bring-up and clean-lap parity

**Deliverable:** the Nav2 stack drives the recorded route as well as the
follower did (R3-N), and still stops on a degraded fix (R5-N).

Runnable instructions: [phase-1-validation.md](./phase-1-validation.md).

**State: written, not built, not run.** See
[progress.md](./progress.md).

## Scope

In: the new package, the Nav2 parameters, the behaviour tree, the mission node,
the launch wiring and the two new harness runs.

Out, deliberately: `route_to_map` and the costmap filters (Phase 2), the
RK3588 CPU measurement and ADR-0004 (Phase 3), safe spots and route schema v2
(Phase 4). `route_follower.py`, `route_recorder.py`, `path.py` and
`route_file.py` are untouched and still work — `r3`, `r4` and `r5` run exactly
as before.

## New package: `src/outdoor_patrol_nav/`

ament_cmake, modelled on `outdoor_patrol_loc`.

```
CMakeLists.txt  package.xml  README.md
include/outdoor_patrol_nav/route_goals.hpp
src/route_goals.cpp          # route reader + subsampler, the only pure code
src/patrol_mission.cpp       # the only new node
config/nav2_params.yaml
config/patrol_mission.yaml
bt/patrol.xml
launch/nav2.launch.py
test/test_route_goals.cpp
test/fixtures/route_square.yaml
```

### `patrol_mission` — the only new node

Roughly 500 lines, covering three things and nothing else:

1. **Project the route.** Read `route_path` with yaml-cpp, wait for
   `/fromLL`, project every sample, subsample every `station_spacing_m` by
   accumulated Euclidean distance. Yaw comes from the recorded field, already
   REP-103. The `/fromLL` calls run from a timer, not the constructor: the
   service needs the executor spinning, which is the trap the follower's own
   comment records.
2. **Send one `NavigateThroughPoses` goal**, `laps` laps of stations, and
   re-send the remaining tail after a degraded-GNSS cancel.
3. **Publish `~/status` and `~/finished`** in the shapes the harness already
   parses.

Degraded-GNSS rule, the one thing Nav2 cannot know:

- Subscribe to raw `/um982_driver/fix`, **not** `/gnss/fix_gated`. The gate
  multiplies covariance by 1000 on a degraded fix — an EKF-weighting device,
  not a quality metric — which would turn the slow/stop pair into a cliff at
  5 cm.
- Horizontal sigma is eight lines inline, with both traps from
  `route_file.py:206` kept as comments: `COVARIANCE_TYPE_UNKNOWN` → infinity,
  `status < 0` → infinity, else `sqrt(max(cov[0], cov[4], 0))`. The **worse**
  axis, because `confidence_gate` tests the same thing and the two must agree
  on what "sigma" means.
- Above `sigma_stop_m` → `async_cancel_goal()`, state `degraded`. Below
  `sigma_slow_m` for `resume_clear_cycles` ticks → re-send.
- **No velocity is published by this node, on any topic.** Nav2 stops
  commanding on cancel and `scan_safety`'s `cmd_timeout_s` (0.5 s) emits the
  zero `Twist` the scorer's `final_speed_ms` reads. Our own zero would be a
  second writer on `/cmd_vel_raw`.

Why C++ and not Python: the decision was "runtime nodes and their libraries in
C++". Launch files and the offline host tools (`score_run.py`,
`score_route.py`, `gen_patrol_road.py`) stay Python.

### What is *not* custom

The first draft of this phase ported `path.py` (332 lines) and `route_file.py`
(256 lines) to C++ so the mission node could compute cross-track error. Reading
`score_run.py` closely killed that: `analyse()` takes exactly three fields off
the status topic — the timestamp, `d_cmd` and `state` — and measures
cross-track, laps, longest stop and clearance from `/odom_truth` against the
world's own centerline YAML, deliberately never from the controller's estimate.

| Job | Would have been | Is |
|---|---|---|
| Smooth the path | port `savitzky_golay` + Catmull-Rom | `nav2_smoother::SavitzkyGolaySmoother` behind the BT's `SmoothPath` |
| Cross-track for status | port `Path.project` | not computed; `d_cmd ≡ 0.0` |
| Speed / accel limits | custom ramp | stock `velocity_smoother` |
| Recoveries | none | stock `behavior_server` |
| lat/lon → map | port of the follower's loop | `robot_localization` `/fromLL`, ~15 lines |

Nothing in `outdoor_patrol_route` is converted. The one place this phase
duplicates Python is the ~40-line route reader — schema version, `loop`,
`samples[].{lat,lon,yaw}`, and the refusal of `source: raw_antenna` with the
follower's own error text. It is not a port of `route_file.py`: no writer, no
`Sample` defaults, no `classify_fix`.

### `config/nav2_params.yaml`

Three of these are walls, not knobs, and the file says so in place:

- `velocity_smoother` limits — the firmware clamps, not preferences
- the footprint polygon — the robot's actual box (see finding 5 in
  [progress.md](./progress.md))
- `minimum_turning_radius: 1.5` — the chassis

The rest, briefly: Smac Hybrid-A* with `REEDS_SHEPP` (finding 4), MPPI as
`FollowPath` and RPP registered alongside as `FollowPathRPP` so Phase 3's
fallback is a one-line change, Savitzky-Golay smoother, both costmaps rolling
with `track_unknown_space: false` and no static layer. No `map_server`, no
costmap filters — GNSS is the map until Phase 2.

The global costmap is 60 × 60 m. That size is a function of **route extent**,
not sensor range: `ComputePathThroughPoses` fails if any goal pose falls
outside the costmap, and a `NavigateThroughPoses` goal covers a whole lap. The
sim's 100 m loop is ~28 m across.

### `bt/patrol.xml`

Stock `navigate_through_poses_w_replanning_and_recovery`, copied in and pinned,
with `SmoothPath` added after `ComputePathThroughPoses`. Pinned because Phase 4
replaces the recovery branch with the safe-spot retreat, and owning the file
makes that a reviewable diff rather than a from-scratch tree. It also stops a
Nav2 upgrade from silently changing what the robot does when it cannot find a
path.

## Modified files

| File | Change |
|---|---|
| `outdoor_patrol_sim/launch/sim.launch.py` | `nav:=true` starts the Nav2 servers off the same `/clock` gate as the EKFs, in a **scoped** `GroupAction` (see that file's own warning about `params_file` leaking between includes). Does not start `patrol_mission`. |
| `outdoor_patrol_sim/scripts/run_validation.sh` | `r3n` and `r5n` cases; a `-N` label switches the follower for `patrol_mission`, the status and finished topics, and adds `nav:=true`. `STACK_NODES` and `reap_stragglers` gain the Nav2 server names. |
| `outdoor_patrol_route/scripts/score_run.py` | `--status-topic`, default unchanged. ~6 lines. The only change to that package, and it is a host-side tool. |

## Acceptance

R3-N and R5-N, both scored by the existing `score_run.py`:

| Run | Gate |
|---|---|
| R3-N | RMS < `NAV_MAX_RMS` (Phase 0), peak < 0.5 m, ≥ 0.98 laps, longest stop < 3 s, stays in the corridor |
| R5-N | ≥ 20 sustained `degraded` cycles **and** `final_speed_ms` ≤ 1e-3 |

There is no `r4n`. R4 scores the retreat from the commanded lateral offset,
which is identically zero under Nav2 (finding 2). Phase 2 replaces that check
before there is an R4-N.

## Done when

- `colcon test --packages-select outdoor_patrol_nav` passes.
- `score_R3-N.json` and `score_R5-N.json` both pass.
- Their numbers are in [progress.md](./progress.md), next to the Phase 0
  baseline.
