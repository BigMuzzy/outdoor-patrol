# RESUME — note to self

Updated 2026-09-09, after Phases 0 and 1 were built, scored in sim, and
**deployed to the robot**. Earlier versions of this file said "nothing is
built, tested or scored" (2026-09-05) and then "done and passing in sim"
(2026-09-06). Both are superseded.

## Where the work stopped

**Phases 0 and 1 are done, passing, and on the robot.** The stack has never
driven outdoors — that is the next thing, and it is blocked on a hardware
fault (see [To continue](#to-continue)).

- Phase 0: 3/3 PASS, mean R3 RMS 0.0645 m, `NAV_MAX_RMS` = 0.129.
- Phase 1: R3-N 0.0883 m RMS / 0.219 m peak / 0.988 laps / 1.68 s longest
  stop; R5-N 452 degraded cycles, final speed 0.0. Both PASS.
- Re-scored after the telemetry additions: R3-N 0.087 m. No regression.
- Deployed: every Nav2 server configures and activates on the RK3588.

Numbers and the full finding list are in [progress.md](./progress.md); the
baseline table is in [runs/baseline/baseline.md](./runs/baseline/baseline.md);
the field procedure and the robot's known faults are in
[field-test-driveway.md](./field-test-driveway.md).

**Branch: `agents/nav2-driveway-field-test`**, cut from `main`, deliberately
isolated from `agents/field-validation-dashboard-rviz`. The older
`agents/read-nav2-migration-doc` was never pushed and does not exist on
origin — do not go looking for it.

> ⚠️ **You probably cannot commit from the devcontainer workspace.**
> `/workspaces/read-nav2-migration-doc` is a git *worktree* whose gitdir lives
> on the host at `/home/max/projects/outdoor-patrol/.git/worktrees/...`, which
> is not mounted into the container. `git status` there fails outright. The way
> through is a plain clone somewhere writable (`git clone
> git@github.com:BigMuzzy/outdoor-patrol.git`), work there, push, and copy
> files back into the workspace for the user to read. SSH auth to GitHub does
> work from inside the container.

## To continue

Ranked. The first is a blocker, not a task.

1. **The IMU is dead and must be fixed before any field run.** `imu_driver`
   logs `No response to command 0x12` three times, then continues anyway, and
   `/imu/data` has **zero publishers**. The FTDI adapter enumerates, so the
   port opens and the cable is intact; the sensor never answers. Untested
   hypothesis: the Inertial Labs KERNEL shares a power rail with the
   drivetrain, which was switched off for the whole deployment session.
   **Check it with the drivetrain powered on** before assuming a driver bug.
   Not optional — the global EKF fuses IMU orientation and `heading_to_imu`
   feeds GNSS heading through the same path, so running without it is a
   different test, not a smaller one.
2. **Phase 0's two field prerequisites have still never been run**: the GNSS
   soak (σ ≤ 0.05 m for 10 min) and `yaw_offset` (heading within 10° of true),
   both in [field-validation-alley.md](../field-validation-alley.md) Phases 1
   and 2. Nav2 depends on both more sharply than the follower did.
3. **The driveway field test**, [field-test-driveway.md](./field-test-driveway.md).
   Record a route with the `record` profile, then follow it with `nav2` +
   `mission`. Everything is deployed and waiting.
4. **Phase 2**, starting with `route_to_map`. Read finding 2 in
   [progress.md](./progress.md) first — the R4 scorer checks have to be
   rewritten before there can be an R4-N, and that is Phase 2's first
   deliverable, not an afterthought.
5. **ADR-0004 can be written now**: parity is measured. Nav2 tracks the clean
   road at 1.37× the follower's cross-track RMS (0.0883 vs 0.0645 m), inside
   the 2× bar. Before quoting 0.0645 as the follower's accuracy, read finding
   9 — a share of it may be the teach driver rather than the follower.

## The robot

Two container sets, two image tags, and they must not run at the same time.

| | This branch | The dashboard branch |
|---|---|---|
| Repo | `~/code/outdoor-patrol-nav2` | `~/code/outdoor-patrol` |
| Image | `outdoor-patrol:nav2` | `outdoor-patrol:arm64` |
| Compose | `deploy/docker-compose.nav2.yaml` | `deploy/docker-compose.yaml` |
| Containers | `outdoor-patrol-nav2`, `-nav2-stack` | `outdoor-patrol`, `-dashboard` |

Separate tags were the point: both of the dashboard branch's containers run
from `outdoor-patrol:arm64`, so building this branch under that tag would have
silently re-imaged them on their next restart. Swap back with `down` in one
directory and `up -d` in the other.

Both sets share **ROS domain 0** and cannot coexist: two bringups is two
`/cmd_vel` publishers and two micro-ROS agents on one serial device.

`deploy/.env` on the robot points `DATA_DIR` at
`/home/ubuntu/code/outdoor-patrol/deploy/data` — the *other* repo's directory.
That is deliberate. `/data` holds `ntrip.yaml`, the recorded routes and the
reports; those belong to the robot, not to a branch. A fresh clone has an
empty `./data`, and the symptom of a missing `ntrip.yaml` is not an error: the
GNSS quietly drops to SPS with no corrections, which looks like poor sky view.

## Environment

Two traps, both of which cost real time.

**`rosdep install` was installing nothing.** `outdoor_patrol_bringup`
exec_depends on `sllidar_ros2`, a submodule absent unless `./setup.sh` has run
with git SSH credentials, and one unresolvable key is fatal for the whole
invocation — so all ~20 other keys were dropped. The symptom reads as "nav2 is
missing". Fixed by `scripts/rosdep-install.sh`, now used by
`devcontainer.json`, `setup.sh` and both stages of `deploy/Dockerfile`.

**The dev box and the robot share ROS domain 0.** `ros2 node list` on the dev
box returns the robot's own nodes, including `/esp32_drive`, which subscribes
to `/cmd_vel` — and a sim run publishes `/cmd_vel`. Run the sim under an
explicit `ROS_DOMAIN_ID` (42 was used) whenever the robot may be powered.
`run_validation.sh`'s stale-node check is what caught this; it is
load-bearing, not housekeeping.

## Files created

```
src/outdoor_patrol_nav/            new package, ament_cmake
  CMakeLists.txt  package.xml  README.md
  include/outdoor_patrol_nav/route_goals.hpp
  src/route_goals.cpp  src/patrol_mission.cpp
  config/nav2_params.yaml  config/patrol_mission.yaml
  config/nav2_params_driveway.yaml  config/patrol_mission_driveway.yaml
  config/field.rviz                  stock displays only, for the field
  config/diagnostics_analyzers.yaml  rqt_robot_monitor grouping
  bt/patrol.xml  bt/patrol_driveway.xml  launch/nav2.launch.py
  test/test_route_goals.cpp  test/fixtures/route_square.yaml
src/outdoor_patrol_sim/worlds/     driveway.sdf  driveway_centerline.yaml
deploy/docker-compose.nav2.yaml    separate tag, separate container names
scripts/rosdep-install.sh          shared, submodule-tolerant rosdep wrapper
doc/eng/plans/nav2-migration/      plan.md progress.md phase-0.md phase-1.md
                                   phase-0-validation.md phase-1-validation.md
                                   field-test-driveway.md RESUME.md
                                   runs/baseline/{README,baseline}.md
                                   runs/baseline/run{1,2,3}/ runs/phase-1/
                                   runs/driveway/README.md
.github/skills/i-have-adhd/SKILL.md   vendored, MIT
```

## Files modified

| File | Change | Risk if wrong |
|---|---|---|
| `outdoor_patrol_sim/launch/sim.launch.py` | `nav:=`, `nav_params_file:=`, `bt_xml:=` | defaults unchanged; existing runs unaffected |
| `outdoor_patrol_sim/scripts/run_validation.sh` | `r3n`/`r5n`, `NAV_MAX_RMS` 0.128 → measured 0.129, and `WORLD`/`CENTERLINE`/`NAV_PARAMS`/`MISSION_PARAMS`/`BT_XML`/`START_*`/`TEACH_SPEED`/`TEACH_LOOKAHEAD` env overrides | every override defaults to the 100 m road, so `r3`/`r4`/`r5` are unchanged |
| `outdoor_patrol_sim/scripts/gen_patrol_road.py` | `--name-prefix` | `--check` confirms the 100 m artefacts are byte-identical |
| `outdoor_patrol_route/scripts/score_run.py` | `--status-topic` | default preserves old behaviour |
| `um982_driver` | dual-antenna diagnostics: `heading_quality`, `heading_deg`, `heading_published`, ANT1/ANT2 satellite counts | recorded *before* the drop check, so a dropped heading stays visible |
| `.devcontainer/devcontainer.json`, `setup.sh`, `deploy/Dockerfile` | all call `scripts/rosdep-install.sh` | skips only keys that come from `src/` |

Nothing in `outdoor_patrol_route` was touched beyond `score_run.py`.
`route_follower.py`, `path.py`, `route_file.py`, `route_recorder.py` all still
work — Phase 0 re-ran `r3`/`r4`/`r5` three times each and they pass.

## What I was least sure about, and how it turned out

The 2026-09-05 ranking, scored against what actually happened. It was a poor
predictor, and that is the lesson: every real defect was in the node's own
control flow, or in a length that was right at one scale and silently wrong at
another — never in the Nav2 parameter set everyone worries about.

1. **`nav2_params.yaml` has never been through `configure()`** — ranked most
   likely. **Wrong twice.** Every server configured and activated first time
   in sim, and again on the RK3588 with the driveway parameter set.
2. **`SmoothPath` BT port names.** **Wrong**, no complaint from
   `bt_navigator`.
3. **`RemovePassedGoals radius` vs `station_spacing_m`.** **Wrong** as
   framed — the robot never circled a station. But the spacing *was* the
   problem, for an unrelated geometric reason (finding 7).
4. **`velocity_smoother` deadband.** Untested in sim; still untested on
   hardware, because nothing has driven yet.
5. **`/plan` topic name.** Fine, populated.

What actually broke: a member variable that did not exist (the package had
never compiled), a lifecycle race on goal acceptance (finding 8), a
closed-loop goal that was complete before the robot moved (finding 6), a
station spacing that could not represent a 5 m corner (finding 7), and the
same class of error again in the teach driver's lookahead (finding 9).

Then, on the robot, three more that only a live bring-up could show:
`rqt_robot_monitor` reads `/diagnostics_agg` and nothing was aggregating; a
fresh clone's empty `/data` silently cost RTK; and `robot_localization`'s
frequency diagnostic reports 0 Hz against a topic measurably running at 30 Hz,
which made the health screen permanently red until it was discarded.

## Decisions already made — do not re-litigate

- Scope **was** Phase 0 + Phase 1, plus enough field telemetry to run the
  driveway test. Phase 2 is the next scope decision.
- **Stock ROS tools over custom UI.** The field readout is RViz with
  `rviz_default_plugins`, `rqt_robot_monitor`, `rqt_plot` and
  `rqt_service_caller` — no bespoke panel. This was a user decision and it
  turned out cheaper: `um982_driver` already published most of the GNSS
  diagnostics, so only the dual-antenna half was new. The dashboard branch's
  `FieldDashboardPanel` is the road not taken here; if a future phase wants a
  panel, that is a reversal to argue for, not a gap to fill.
- Runtime nodes and their libraries in C++; launch files and offline host
  tools (`score_run.py`, `score_route.py`, `gen_patrol_road.py`) stay Python.
- C++ ports are all-or-nothing: nothing in `outdoor_patrol_route` is
  converted, and the ~40-line route reader in `route_goals.cpp` is a reader,
  not a port of `route_file.py`.
- `patrol_mission` uses stock Nav2 wherever a stock component exists. Chunked
  goal dispatch (finding 6) is the one place Phase 1 had to add mission-level
  sequencing, and it sequences *stock* goals rather than replacing one.
- Live `cross_track_m` is a **field instrument, not a gate**. It is measured
  against the recorded route using the robot's own estimate, so it cannot see
  an error that moves the route and the robot together. `score_run.py` remains
  the thing that judges a run.
