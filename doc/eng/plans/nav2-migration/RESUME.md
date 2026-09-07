# RESUME — note to self

Updated 2026-09-06, after Phases 0 and 1 were built, run and scored. The
original version of this file was written on 2026-09-05 in an environment with
no ROS, and said "nothing is built, tested or scored". That is no longer true.

## Where the work stopped

**Phases 0 and 1 are done and passing.** Numbers and the full finding list are
in [progress.md](./progress.md); the baseline table is in
[runs/baseline/baseline.md](./runs/baseline/baseline.md).

- Phase 0: 3/3 PASS, mean R3 RMS 0.0645 m, `NAV_MAX_RMS` = 0.129.
- Phase 1: R3-N 0.0883 m RMS / 0.219 m peak / 0.988 laps / 1.68 s longest
  stop; R5-N 452 degraded cycles, final speed 0.0. Both PASS.

Branch: `agents/read-nav2-migration-doc`.

## To continue

1. Phase 2, starting with `route_to_map`. Read finding 2 in
   [progress.md](./progress.md) first — the R4 scorer checks have to be
   rewritten before there can be an R4-N, and that is Phase 2's first
   deliverable, not an afterthought.
2. ADR-0004 can now be written: parity is measured. Nav2 tracks the clean road
   at 1.37× the follower's cross-track RMS (0.0883 vs 0.0645 m), inside the 2×
   bar, which is the trade the decision record needs to state.
3. Finding 7 is the one to carry into the field. `station_spacing_m: 2.0` is
   derived from the sim road's 5 m corner radius; a real site with tighter
   corners needs a smaller number and nothing checks it.

## Environment

Not a footnote — it cost the first hour. `rosdep install` was exiting
non-zero having installed **nothing**, because `outdoor_patrol_bringup`
exec_depends on `sllidar_ros2`, a submodule that is absent unless `./setup.sh`
has run with git SSH credentials, and one unresolvable key is fatal for the
whole invocation. The symptom reads as "nav2 is missing".

Fixed by `scripts/rosdep-install.sh`, now used by `devcontainer.json`,
`setup.sh` and both stages of `deploy/Dockerfile`. A rebuilt container gets
Nav2 without manual steps.

## Files created

```
src/outdoor_patrol_nav/            new package, ament_cmake
  CMakeLists.txt  package.xml  README.md
  include/outdoor_patrol_nav/route_goals.hpp
  src/route_goals.cpp  src/patrol_mission.cpp
  config/nav2_params.yaml  config/patrol_mission.yaml
  bt/patrol.xml  launch/nav2.launch.py
  test/test_route_goals.cpp  test/fixtures/route_square.yaml
scripts/rosdep-install.sh          shared, submodule-tolerant rosdep wrapper
doc/eng/plans/nav2-migration/      plan.md progress.md phase-0.md phase-1.md
                                   phase-0-validation.md phase-1-validation.md
                                   RESUME.md runs/baseline/{README,baseline}.md
                                   runs/baseline/run{1,2,3}/ runs/phase-1/
.github/skills/i-have-adhd/SKILL.md   vendored, MIT
```

## Files modified

| File | Change | Risk if wrong |
|---|---|---|
| `outdoor_patrol_sim/launch/sim.launch.py` | `nav:=` argument, Nav2 off the `/clock` gate | default is `false`; existing runs unaffected |
| `outdoor_patrol_sim/scripts/run_validation.sh` | `r3n`/`r5n`, `-N` label switching, `STACK_NODES`, `NAV_MAX_RMS` 0.128 → measured 0.129 | `r3`/`r4`/`r5` paths unchanged apart from the new `--status-topic` flag |
| `outdoor_patrol_route/scripts/score_run.py` | `--status-topic`, default `/route_follower/status` | default preserves old behaviour |
| `.devcontainer/devcontainer.json`, `setup.sh`, `deploy/Dockerfile` | all three call `scripts/rosdep-install.sh` | skips only keys that come from `src/` |

Nothing else in `outdoor_patrol_route` was touched. `route_follower.py`,
`path.py`, `route_file.py`, `route_recorder.py` all still work and `r3`, `r4`,
`r5` still run — Phase 0 re-ran all three, three times, and they pass.

## What I was least sure about, and how it turned out

The 2026-09-05 ranking, scored against what actually happened. It was a poor
predictor, which is itself worth knowing: every real defect was in the node's
own control flow, not in the Nav2 configuration everyone worries about.

1. **`nav2_params.yaml` has never been through `configure()`** — ranked most
   likely. **Wrong.** Every server configured and activated first time. The
   MPPI critic names and `SmacPlannerHybrid`'s spelling were both fine.
2. **`SmoothPath` BT port names.** **Wrong**, no complaint from
   `bt_navigator`.
3. **`RemovePassedGoals radius` vs `station_spacing_m`.** **Wrong** as
   framed — the robot never circled a station. But the spacing *was* the
   problem, for an unrelated geometric reason (finding 7).
4. **`velocity_smoother` deadband.** Untested in sim, still open on hardware.
5. **`/plan` topic name.** Fine, populated.

What actually broke: a member variable that did not exist (the package had
never compiled), a lifecycle race on goal acceptance (finding 8), and a
closed-loop goal that was complete before the robot moved (finding 6).

## Decisions already made — do not re-litigate

- Scope **was** Phase 0 + Phase 1. Both are now done; Phase 2 is the next
  scope decision, not a continuation of this one.
- Runtime nodes and their libraries in C++; launch files and offline host
  tools (`score_run.py`, `score_route.py`, `gen_patrol_road.py`) stay Python.
- C++ ports are all-or-nothing: nothing in `outdoor_patrol_route` is
  converted, and the ~40-line route reader in `route_goals.cpp` is a reader,
  not a port of `route_file.py`.
- `patrol_mission` uses stock Nav2 wherever a stock component exists. If a
  Phase 2 task looks like it needs new custom code, check whether a Nav2
  plugin already does it before writing it. Chunked goal dispatch (finding 6)
  is the one place Phase 1 had to add mission-level sequencing, and it is
  sequencing *of* stock goals rather than a replacement for one.

