# Driveway field test — Nav2, with stock tools

First time the Nav2 stack drives a real robot. Everything you watch is a
stock ROS tool: RViz with `rviz_default_plugins`, `rqt_robot_monitor`,
`rqt_plot`, `rqt_service_caller`. There is no custom panel to build.

**Status: deployed and brought up on the robot 2026-09-06; not yet driven.**
The stack is on the Pi and every Nav2 server activates — see
[Deployment result](#deployment-result). What has *not* happened is a route
being recorded or followed on real ground.

## Before you go outside

Two Phase 0 field prerequisites in [plan.md](./plan.md) have **no field
result yet**, and Nav2 depends on both:

- GNSS soak, σ ≤ 0.05 m for 10 minutes with no dropouts
- `yaw_offset` — heading within 10° of true

Neither is a formality for this stack. `patrol_mission` cancels the goal
above `sigma_stop_m` (0.50 m), and Nav2 plans in the `map` frame that the
navsat datum defines — a bad heading rotates the whole route about the datum
and the robot will drive a correctly-shaped loop in the wrong place.
[field-validation-alley.md](../field-validation-alley.md) Phases 1 and 2
cover both. Do them first.

## Deploy

The robot's two existing containers (`outdoor-patrol`,
`outdoor-patrol-dashboard`) come from a different branch and run from the
`outdoor-patrol:arm64` tag. This branch builds `outdoor-patrol:nav2` with
`*-nav2` container names, so the two sets cannot overwrite each other — but
they still **share ROS domain 0** and cannot run at the same time. Two
bringups means two `/cmd_vel` publishers and two micro-ROS agents on one
serial device.

```bash
ssh robot
cd ~/code/outdoor-patrol && docker compose -f deploy/docker-compose.yaml down
cd ~/code/outdoor-patrol-nav2
docker compose -f deploy/docker-compose.nav2.yaml build      # ~9 min
docker compose -f deploy/docker-compose.nav2.yaml up -d robot
```

To go back afterwards: `docker compose -f deploy/docker-compose.nav2.yaml
down` in `outdoor-patrol-nav2`, then `up -d` in `outdoor-patrol`.

## Deployment result

Done 2026-09-06. Image `outdoor-patrol:nav2` (4.1 GB) built on the Pi in
~28 min — longer than the ~9 min the deploy skill quotes, because Nav2 and
its dependencies were a cold layer. `outdoor-patrol:arm64` was left
untouched, so the other branch's containers can be brought back by starting
them.

**Every Nav2 server configured and activated on the RK3588 first time:**
`controller_server` (MPPI + `FollowPathRPP`), `planner_server`
(`SmacPlannerHybrid`), `smoother_server`, `behavior_server`, `bt_navigator`,
`velocity_smoother` — all bonded to the lifecycle manager, with
`/navigate_through_poses` and both costmaps on the graph. That closes the
top-ranked risk in [RESUME.md](./RESUME.md) ("`nav2_params.yaml` has never
been through `configure()`") on hardware as well as in sim, and it is the
first evidence that `nav2_params_driveway.yaml` is valid outside Gazebo.

The GNSS telemetry reads correctly on the real receiver:

| Field | Value |
|---|---|
| `fix_quality` | 4 (RTK fixed) |
| `num_satellites_ant1_position` | 23 |
| `num_satellites_ant2_heading` | 25 |
| `heading_quality` | 2 (RTK float baseline) |
| `heading_deg` | 166.17 |
| `hdop` | 0.6 |

**Not measured:** MPPI CPU under load. The controller was activated but
never given a goal, so it was not optimising; the idle figures say nothing
about the Phase 3 question. Take that reading during the first followed lap.

## The four windows on the dev box

```bash
# 1. Spatial view. Route, plans, costmaps, scan.
ros2 launch outdoor_patrol_bringup rviz.launch.py \
  rviz_config:=$(ros2 pkg prefix outdoor_patrol_nav)/share/outdoor_patrol_nav/config/field.rviz

# 2. Health. GNSS fix quality, satellite counts, ANT2 heading, mission state.
ros2 run rqt_robot_monitor rqt_robot_monitor

# 3. Numbers over time.
ros2 run rqt_plot rqt_plot \
  /patrol_mission/cross_track_m/data /patrol_mission/speed_mps/data

# 4. Buttons. Save/discard the recorded route.
ros2 run rqt_service_caller rqt_service_caller
```

### What `rqt_robot_monitor` shows

Under **um982_driver**:

| Field | Meaning |
|---|---|
| `fix_quality` | 4 = RTK fixed, 5 = float, 1 = single. Below 4, expect metre-level error |
| `num_satellites` | ANT1 position solution |
| `num_satellites_ant1_position` | same, from KSXT |
| `num_satellites_ant2_heading` | **ANT2** — the heading antenna |
| `heading_quality` | 0 = baseline unsolved, 1 = single, 2 = float, 3 = fixed |
| `heading_deg` | last heading, whether or not it was published |
| `heading_published` | false = the heading was DROPPED |
| `hdop`, `correction_age_s` | geometry and RTCM freshness |

`heading_quality: 0` with a healthy `fix_quality` is the ANT2 signature —
see [heading-wrong-ant2-no-signal.md](../../wiki/gnss/heading-wrong-ant2-no-signal.md).
The heading fields are recorded **even when the heading is dropped**, which
is the case you most need to see.

Under **patrol_mission**: `state`, `cross_track_m`, `cross_track_max_m`,
`poses_remaining`, `sigma_h_m`.

## Record a route

The recorder starts recording the moment it comes up.

```bash
# on the robot
docker compose -f deploy/docker-compose.nav2.yaml --profile record up -d
```

Drive the loop with teleop, ending where you started (the route is recorded
`loop: true`). Then, from `rqt_service_caller` or the command line:

```bash
ros2 service call /route_recorder/save std_srvs/srv/Trigger
```

It lands in `/data/routes/driveway.yaml` on the robot's own disk, which is
where `patrol_mission` reads it from — nothing to copy. `ROUTE_NAME` in
`deploy/.env` changes the filename. Then stop the recorder:

```bash
docker compose -f deploy/docker-compose.nav2.yaml stop recorder
```

**Drive it as accurately as you want it repeated.** Finding 9 in
[progress.md](./progress.md) is the sim version of this: the follower was
tracking a badly-taught route to within 2 mm and the error looked like a
following error. The same applies here — `cross_track_m` is measured against
what you recorded, so a sloppy teach pass reports as a clean repeat.

## Follow it

⚠️ **The `mission` service moves the robot.** It starts driving as soon as
`bt_navigator` is active. Bring up `nav2` first and confirm the costmaps look
sane in RViz, ideally with a manual 2D Goal Pose, before starting `mission`.

```bash
docker compose -f deploy/docker-compose.nav2.yaml --profile nav2 up -d nav2
# check RViz: costmaps present, robot in the right place, no inflation
# covering the whole driveway
docker compose -f deploy/docker-compose.nav2.yaml --profile nav2 up -d mission
```

Hand on the kill switch. `scan_safety` still sits between `/cmd_vel_raw` and
`/cmd_vel` exactly as it does under the old follower (ADR-013), and Nav2 does
not bypass it.

## What "it worked" looks like

There is no scored gate here — `score_run.py` needs Gazebo ground truth, and
outdoors there is none. Judge it on:

- `cross_track_max_m` stays inside the corridor you recorded
- `state` never reaches `blocked`
- the robot completes the loop without a recovery behaviour firing
- `heading_quality` stays ≥ 1 throughout — a dropped heading mid-run is the
  most likely cause of a sudden excursion

Record the numbers here afterwards. `cross_track_m` is **not** comparable to
the sim's 0.0883 m: it is measured against the recorded route using the
robot's own estimate, so it cannot see an error that moves the route and the
robot together. It is a field instrument, not a score.

## If it fails

**Robot does not move, no error.** Check `ros2 topic info /cmd_vel_raw` is
`geometry_msgs/msg/Twist`. Then check `scan_safety` is not braking on a
phantom return — the driveway is narrower than the alley and a wall inside
the brake distance stops the robot legitimately.

**`waiting for /fromLL`.** `navsat_transform` is not up, or the datum is not
satisfied. This is the Phase 0 prerequisite biting.

**Route drives in the wrong place.** Heading. Check `heading_quality` and the
`yaw_offset` from field validation Phase 2.

**Robot circles a station.** `RemovePassedGoals radius` in
`bt/patrol_driveway.xml` (0.25 m) versus `station_spacing_m` (0.6 m) in
`config/patrol_mission_driveway.yaml`. The radius must stay under the
spacing.

**Everything is at the origin in RViz.** The `map` frame is not established;
`/diagnostics` first, not RViz.
