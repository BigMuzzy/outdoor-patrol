# RESUME — note to self

Updated 2026-09-09 UTC after simulation-first replanning for Jetson Orin Nano
and monocular camera perception. Phases 0 and 1 were built, scored in sim,
and deployed to the previous RK3588 setup. **The onboard PC is now removed:
do not start field work or treat that deployment as the current target.**

For the checked-in node/topic/TF graph and configuration caveats, read
[Current ROS configuration](../../ros-configuration.md), with editable diagrams.

## Where the work stopped

**Phases 0 and 1 passed in simulation; the stack has never driven outdoors.**
Continue in simulation. Field gates are deferred until the hardware returns,
not prerequisites for further sim work and not waived by it.

The revised [plan.md](./plan.md) targets:

- Jetson Orin Nano (advertised up to 67 TOPS; exact configuration TBD).
- One RGB camera with learned monocular depth and terrain segmentation.
- Dual-antenna GNSS, 2D lidar and the **retained IMU**, with existing chassis
  odometry, motor control, watchdog and operator stop.
- Predefined-route patrol, bounded obstacle detours and **predefined safe
  spots for yielding to traffic**. Stopping in the lane is not a successful
  yield; the camera must not invent permission to drive on unknown ground.
- Community packages first: stock Nav2 navigation/BT components, standard
  filters and Collision Monitor, existing perception models/wrappers.
  Only narrowly scoped mission/perception integration code if reuse leaves
  a demonstrated gap.

- Phase 0: 3/3 PASS, mean R3 RMS 0.0645 m, `NAV_MAX_RMS` = 0.129.
- Phase 1: R3-N 0.0883 m RMS / 0.219 m peak / 0.988 laps / 1.68 s longest
  stop; R5-N 452 degraded cycles, final speed 0.0. Both PASS.
- Re-scored after the telemetry additions: R3-N 0.087 m. No regression.
- Historical deployment: every Nav2 server configured and activated on the
  RK3588. No Jetson timing or camera accuracy measurements exist yet.

Numbers and the full finding list are in [progress.md](./progress.md); the
baseline table is in [runs/baseline/baseline.md](./runs/baseline/baseline.md);
the field procedure and the robot's known faults are in
[field-test-driveway.md](./field-test-driveway.md).

**Branch: `agents/nav2-driveway-field-test`**, cut from `main`, deliberately
isolated from `agents/field-validation-dashboard-rviz`. The older
`agents/read-nav2-migration-doc` was never pushed and does not exist on
origin — do not go looking for it.

> ⚠️ **Git does not work in the `read-nav2-migration-doc` devcontainer.**
> `/workspaces/read-nav2-migration-doc` is a git *worktree* whose gitdir lives
> on the host at `/home/max/projects/outdoor-patrol/.git/worktrees/...`. Only
> the working tree is bind-mounted into the container, not that path, so git
> cannot resolve its own metadata and `git status` fails outright. SSH auth to
> GitHub *does* work from inside the container — it is only the local repo
> that is unreachable. See [Opening it so git works](#opening-it-so-git-works).

## Opening it so git works

The branch is on origin, so the fix is to open a checkout whose `.git` is
reachable from inside the container. Two ways, in order of preference.

**1. Open the main repo instead of the worktree.** Its `.git` is a real
directory inside the folder VS Code mounts, so git works with no
configuration at all. On the **host**:

```bash
cd ~/projects/outdoor-patrol
git fetch origin
git switch agents/nav2-driveway-field-test
code .
```

Then *Reopen in Container*. Cost: a fresh `build/`+`install/`, so budget
~5 minutes for `./build.sh`. The worktree has served its purpose and can be
retired with `git worktree remove read-nav2-migration-doc` once you are sure
nothing local is left in it — as of 2026-09-09 its contents are byte-identical
to the branch, so nothing would be lost.

**2. Keep this folder and mount the gitdir.** Add to `mounts` in
`.devcontainer/devcontainer.json`, then rebuild the container:

```json
"source=${localEnv:HOME}/projects/outdoor-patrol/.git,target=/home/max/projects/outdoor-patrol/.git,type=bind,consistency=cached"
```

The `target` must match the path in the `.git` file *literally* — git follows
it as an absolute path. Keep this change **local and uncommitted**: the bind
source has to exist on the host or the container will not start, so committing
it would break the container for anyone whose clone is elsewhere.

Either way, a fresh clone (`git clone
git@github.com:BigMuzzy/outdoor-patrol.git`) anywhere writable also works and
is what the deployment work was actually done from.

## To continue

Phases 2 onward are renumbered in the revised [plan.md](./plan.md); older
phase references in historical notes describe the superseded plan.

1. **Phase 2: static detours and the stop chain in sim.** Start with finding 2
   in [progress.md](./progress.md): R4-N needs ground-truth obstacle scoring,
   not the follower's commanded-offset assertions. Use a standard
   corridor/bay-mask fixture and stock Nav2 filters before deciding whether
   an offline `route_to_map` converter is needed. Validate Collision Monitor
   early without bypassing the existing brake; no unvalidated reverse/spin.
2. **Phase 3: predefined bay entry, waiting and rejoin**, using stock Nav2
   goals and sim-only oracle traffic events. Test mission policy separately
   from perception. Bay selection must respect approved access, visibility
   and time available, not just "nearest spot behind".
3. **Phases 4/5: actual RGB models and camera-triggered yielding.** Audit
   supported Jetson/JetPack/ROS/model packages first. Keep ideal simulator
   depth/labels separate from learned model inputs. Metric monocular depth,
   terrain-to-costmap integration and traffic-clearance evidence are not
   assumed solved by installing a network.
4. **Hardware/field gates are deferred to Phase 6.** The GNSS soak
   (σ ≤ 0.05 m for 10 min) and heading within 10° of true in
   [field-validation-alley.md](../field-validation-alley.md) still apply,
   followed by camera calibration, full-stack Jetson timing, actual braking
   and the supervised [driveway test](./field-test-driveway.md). Do not
   connect/deploy/drive while the onboard PC is removed.
5. **ADR-0004 can be written now**: parity is measured. Nav2 tracks the clean
   road at 1.37× the follower's cross-track RMS (0.0883 vs 0.0645 m), inside
   the 2× bar. Before quoting 0.0645 as the follower's accuracy, read finding
   9 — a share of it may be the teach driver rather than the follower.

### Resolved: the IMU was unpowered

Kept because the signature is misleading. On 2026-09-06 `imu_driver` logged
`No response to command 0x12` and `/imu/data` had zero publishers, while the
FTDI adapter enumerated normally — which looks exactly like a driver or
baud-rate bug, and the driver warns and then continues rather than failing.
It was power: the Inertial Labs KERNEL shares a rail with the drivetrain,
which was off. The operator confirmed on 2026-09-09 that the IMU works with
power restored.

Not re-verified from the dev box, because the robot was powered down again
first. On the next power-up, before recording a route:

```bash
ros2 topic hz /imu_driver/data     # current source topic; expect ~100 Hz
```

## The previous RK3588 deployment (historical)

Retained for eventual recovery, not Jetson deployment instructions. The
onboard PC is removed; these paths and images have not been qualified for
the replacement hardware.

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

- Scope is now simulation-first development against the revised roadmap.
  Phase 0/1 results stay frozen; the immediate next implementation is Phase 2.
  No field operation until the replacement hardware is qualified.
- **Reuse before new code.** Stock Nav2 actions, costmap filters and safety
  components, existing perception models and standard ROS interfaces.
  Predefined traffic bays are required. Monocular depth does not authorize
  unrecorded off-road detours.
- **Stock ROS tools over custom UI.** The field readout is RViz with
  `rviz_default_plugins`, `rqt_robot_monitor`, `rqt_plot` and
  `rqt_service_caller` — no bespoke panel. This was a user decision and it
  turned out cheaper: `um982_driver` already published most of the GNSS
  diagnostics, so only the dual-antenna half was new. The dashboard branch's
  `FieldDashboardPanel` is the road not taken here; if a future phase wants a
  panel, that is a reversal to argue for, not a gap to fill.
- New robot-specific runtime nodes and libraries stay C++; launch files and
  offline host tools (`score_run.py`, `score_route.py`, `gen_patrol_road.py`)
  stay Python. Reuse upstream packages without rewriting them just to match
  this language preference.
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
