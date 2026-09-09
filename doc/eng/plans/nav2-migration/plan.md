# Implementation plan - Nav2 patrol, simulation first

**Revised:** 2026-09-09 UTC. Replaces the unimplemented phases of the
2026-09-05 RK3588 migration plan. Phases 0 and 1 and their recorded results
remain historical evidence, not claims about the new hardware.
See [progress.md](./progress.md) and [RESUME.md](./RESUME.md).

## Mission and confirmed decisions

An outdoor, open-sky robot follows a predefined route, detours around
obstacles inside approved drivable areas, and **pulls into predefined safe
spots to let approaching low-speed traffic pass**, then resumes its route.
Stopping in the traffic lane is not a successful yield.

The intended hardware is:

- NVIDIA Jetson Orin Nano, advertised at up to 67 TOPS; exact module,
  memory, carrier, power mode and cooling still need confirming.
- One RGB camera with **learned monocular depth**, not stereo or an RGB-D
  sensor; terrain perception needs spatial segmentation, not just an
  image-wide "grass/road" label.
- Dual-antenna GNSS receiver and 2D lidar.
- The existing IMU is retained. Existing wheel odometry, motor control,
  firmware limits, watchdog and operator stop remain part of the architecture.

The onboard PC is removed, so field work is deferred. Simulation development
does not wait for it. Before hardware driving resumes, the original GNSS soak
and heading checks, new camera calibration, braking tests and target-compute
gates must pass; simulation cannot waive them.

**Prefer maintained community packages and ROS standards over custom code.**
Do not build a new controller, planner, EKF, depth network, object tracker,
3D mapper or dashboard. First try upstream packages through configuration.
Small integration code is acceptable only for a demonstrated gap, with an
explicit interface and tests. "No custom code" is not an honest promise for
the complete camera-to-terrain-to-yield mission.

## Reassessment

**Keep Nav2.** Phase 1 measured R3-N at 0.0883 m RMS against a frozen
0.0645 m follower baseline, within the 0.129 m parity bar; R5-N passed.
There is no evidence that replacing navigation again would help.

Change the unfinished plan in these ways:

| Old assumption | Revised decision |
|---|---|
| Every new phase waits for an outdoor gate | Separate simulation acceptance from deferred hardware/field acceptance. |
| RK3588 CPU phase decides the architecture | Keep CPU Nav2; benchmark the complete CPU/GPU stack on the actual Jetson later. TOPS is not a Nav2 timing guarantee. |
| Camera is a late vehicle-detection add-on | Define depth, terrain and traffic interfaces early; introduce real models after deterministic navigation/yield tests. |
| Custom `scan_tracker` and several custom BT nodes | Start with stock Nav2 actions/BT nodes and conservative traffic-presence events; audit upstream integrations before adding only necessary mission glue. |
| Nearest spot behind the robot, reverse allowed | Select a permitted, reachable bay by route connectivity, visibility and time available; do not reverse blindly. |
| Automatically cut a connector to an off-route spot | Survey/author the bay and its access area explicitly. A geometric connector is not proof that ground is drivable. |
| Obstacle clearing implies moving-traffic handling | A costmap is not a predictor. Early yielding, occupancy of the passing area and safe re-entry are mission requirements. |
| Add Collision Monitor after vehicle detection | Validate it before detours and bay maneuvers; keep the existing brake until equivalence is demonstrated. |
| Camera promises 50-100 m detection | No range promise before camera/model measurements. Visibility, bay spacing and allowed traffic speed must agree. |
| Remove the follower after the compute phase | Keep it as a regression/reference option until replacement field validation is complete. |

## Architecture and reuse boundary

```text
GNSS + dual-antenna heading + IMU + wheel odometry
    -> robot_localization / navsat_transform -> map / odom / base_link

recorded route + approved corridor/bay maps
    -> existing patrol_mission -> stock Nav2 navigation actions
    -> planner + controller + velocity smoother
    -> validated safety chain -> firmware watchdog / motor controller

2D lidar --------------------> obstacle costmaps + independent stop path
RGB -> existing depth model --> qualified depth geometry --+
RGB -> existing segmentation -> terrain evidence ---------+-> local constraints
RGB -> existing detections --> conservative traffic events -> mission yield policy
```

GNSS supplies localization, **not a traversability map**. Keep a separation
between the recorded route (preferred progress), approved areas (where the
robot is permitted to go), observed obstacles and terrain evidence.

### Package choices

| Function | Starting choice | Boundary |
|---|---|---|
| Localization | `robot_localization`, `navsat_transform_node`, existing GNSS/IMU drivers | Retain quality/freshness gates; dual GNSS heading does not replace gravity/tilt measurements. |
| Navigation | Existing Nav2, Smac Hybrid-A*, MPPI, stock navigation actions and BT nodes | Keep the scored configuration initially; RPP remains an unproven fallback, not an automatic performance-equivalent switch. |
| Map constraints | Nav2 map server, keepout and speed filters | Use standard masks; author maps first. Add a small offline route-to-mask converter only if needed, not a new mapping stack. |
| Lidar/depth obstacles | Nav2 obstacle/voxel layers; standard image/point-cloud processing | Depth must have calibrated units, camera geometry and valid timestamps before becoming metric obstacles. Ground filtering is required. |
| Independent reactive stop | Nav2 Collision Monitor plus retained `scan_safety` during validation | Neither is a certified safety system. Validate timeout, reverse and turning coverage before enabling maneuvers. |
| Learned perception | Existing pretrained depth, segmentation and detection models; supported ROS inference wrappers | Model/runtime selection is a compatibility and accuracy gate, not a commitment to train custom networks. |
| Yield sequencing | Existing `patrol_mission` using stock `NavigateToPose` / `NavigateThroughPoses` | Bay policy, traffic event adapter and rejoin bookkeeping are the likely small custom boundary. No new navigation action implementation. |
| Inspection and recording | RViz, diagnostics, rosbag2, existing scoring harness | No custom UI. Keep ground truth out of runtime navigation. |

Use REP-103/105 frames, calibrated `sensor_msgs/Image` and `CameraInfo`,
`PointCloud2` for qualified geometry, `nav_msgs/OccupancyGrid` for maps and
`vision_msgs` detections where the chosen packages support them. Record
sensor time, frames, validity and source; do not disguise uncalibrated depth
or a 2D bounding box as a reliable 3D position.
Depth projection also needs calibrated gravity/tilt information; a planar
navigation pose must not be mistaken for measured camera roll and pitch.

The current 100 m configuration uses `REEDS_SHEPP`, 2 m stations and chunked
goals, not the original plan's invalid `DIFF` setting or 10 m stations.
Its 1.5 m planner turning radius is a tuning choice, not a chassis minimum.
See progress findings 4, 6, 7 and 10. Do not change these settings merely
because the computer changes.

### Camera and terrain policy

Learned monocular depth is an estimate. Even a model trained for metric depth
can have scale bias, unstable edges and out-of-domain errors. Benchmark it on
the actual camera and surfaces. A lidar scan can cross-check visible surfaces
at its scan plane; it cannot validate the whole image, low obstacles,
overhangs, slopes or drop-offs.

Initially, camera evidence may **restrict** or slow travel within approved
areas; it must not open a previously forbidden shoulder. A "grass" label
does not prove load-bearing ground, and missing/invalid depth is not free
space. Combine semantic evidence with geometry, visibility and uncertainty.
Positive lidar obstacles and fixed keepouts must not be erased by a camera
prediction or costmap-clearing recovery.

Nav2 can plan through an approved, not-yet-observed part of a route, but
execution must remain inside the currently verified stopping envelope.
Occluded or expired terrain evidence cannot authorize a new detour.
Do not silently treat camera failure as a lidar-only continuation in a
segment whose terrain/traffic safety depends on the camera.

A semantic image-to-ground constraint integration is not provided merely by
installing a segmentation network. Verify a maintained, compatible adapter;
otherwise keep semantic output advisory and explicitly defer camera-driven
terrain decisions, or approve a narrowly scoped adapter. Do not describe
this integration as finished by pointing at a colored RViz image.

**Concrete reuse shortlist, not yet selected or tested in this repository:**

| Candidate | What it avoids writing | Qualification still needed |
|---|---|---|
| [Isaac ROS Image Segmentation / Segformer](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_segmentation) | GPU inference and ROS segmentation plumbing | Suitable terrain classes/weights, model license, latency and accuracy on the selected camera. |
| [Depth Anything V2 outdoor metric models](https://github.com/DepthAnything/Depth-Anything-V2/tree/main/metric_depth) | Training a monocular depth network from scratch | Metric checkpoint rather than relative depth; compatible inference wrapper/export, license, memory, field error and temporal stability. An outdoor training set is not validation on this route. |
| [kiwicampus semantic_segmentation_layer](https://github.com/kiwicampus/semantic_segmentation_layer) | A new semantic Nav2 costmap plugin | Its documented input is a segmentation mask plus an aligned per-pixel XYZ point cloud, originally for RGB-D sensors. Test the chosen ROS release, cost-combination/decay behavior and learned-depth input; it is not Nav2 core or a certified terrain validator. |
| ROS `depth_image_proc` | A bespoke calibrated depth-to-point-cloud projection | Valid metric depth encoding, matching intrinsics/resolution and alignment with the segmentation image; this does not ground-filter or validate the depth model. |

Evaluate this composition before designing a new semantic layer or projection
node. Package availability is evidence of reusable building blocks, not proof
of a turnkey, qualified outdoor monocular navigation system.

### Pull-over policy

Predefine each bay's usable polygon, approved access area, entry/wait/rejoin
poses, applicable route segments, permitted approach direction and visibility
requirements. A point and a fixed-radius island are insufficient.

Use stock navigation goals to enter and leave a bay. One mission owner
arbitrates actions: cancel/acknowledge the patrol goal, enter the bay, wait
for positive evidence of clearance, rejoin and continue from recorded route
progress. Repeated detections must not restart an in-progress yield.
Localization failure, blocked access and missing required observations lead
to an explicit stopped/fault state, not a successful mission result.

For the first traffic policy, conservatively yield to relevant traffic
presence using a declared maximum approach speed. Reliable velocity/TTC
estimation is not required to build a new tracker first. This trades some
false yields for less custom code; it does **not** remove the need to measure
detection range, latency and the complete time to reach a bay.

For every shared segment, check a conservative feasibility bound:

```text
T_available = (verified_detection_distance - separation_margin)
              / (robot_max_speed + traffic_max_speed)
T_required  = sensing_and_decision_latency + bounded_time_to_bay + time_margin

require T_available > T_required
```

This is an admission check, not a trajectory predictor: the swept access path
must also clear the traffic path before arrival. If the bound or visibility
does not hold, shorten bay spacing, lower the admitted speed, change the
route or improve sensing. More TOPS cannot fix a hidden bend or an
unreachable bay. A late-detection stop is a mitigation, not a passing yield.

Start with forward-access bays. Reverse or spin requires separately validated
swept-footprint coverage and approved terrain; a forward camera and a
forward-only brake do not provide that. No automatic "nearest bay behind me"
rule. A free geometric path is not proof that oncoming traffic has passed;
occlusion, detector loss and a timeout alone must not authorize re-entry.

## Revised phases

Phases below replace the old numbering from Phase 2 onward. A simulation pass
permits further simulation work, not deployment. Keep hardware gates marked
**deferred**, not passed. Detailed Phase 0/1 documents remain historical.

### Phase 0/1 - preserve the baseline and measured Nav2 parity

Done in simulation; deployed once to RK3588, never driven outdoors.
Preserve the frozen runs and thresholds. Finding 9's teach-driver bias merits
a separately labeled baseline experiment, not overwriting the reference.

### Phase 2 - bounded static detours and the stop chain

1. Rewrite/add the Nav2 R4 scorer first (progress finding 2). Use world
   geometry and the full footprint, not commanded `d_cmd`, for clearance and
   lateral excursion. Preserve existing follower scoring.
2. Author a small corridor/bay mask fixture with Nav2 map server and filters.
   Verify datum alignment, bounds, resolution, full-footprint exclusion and
   keepout margins in both costmaps. A filter alone is not a physical fence.
   Use existing world artefacts; build `route_to_map` only if manual standard
   masks are insufficient for repeatable route conversion.
3. Introduce Collision Monitor through one serial velocity path, downstream
   of smoothing, with exactly one final `/cmd_vel` publisher. Retain the
   existing brake and firmware contract until equivalence is tested; do not
   create parallel stop publishers. Disable unvalidated spin/backup recoveries.
   Configure required-source timeouts explicitly, including partial source
   loss. Nav2 documents stopping on stale source data; setting
   `source_timeout: 0.0` disables that protection. Verify behavior on the
   pinned Jazzy package rather than copying rolling defaults or assuming
   another healthy sensor makes the missing one optional.
4. Validate static detours and a full blockage with no available safe path.

**Sim gates:** R3-N/R5-N remain passing. R4-N retains >0.15 m obstacle
clearance, approved-corridor containment, return to |d| <0.2 m within 10 m,
lap completion and longest stop <3 s on the existing partial-obstacle
100 m world. Check full-body containment, not just the center. No
retreat-side assertion. Full-block and traffic-yield scenarios have separate
expected-wait criteria; a full block must stop safely.
Inject stale scan, stale velocity, lifecycle restart and costmap-clearing
events; measure command latency and actual stopping distance independently.

Stopping envelopes must include `v * latency + v^2 / (2 * minimum_deceleration)`
plus footprint/uncertainty margins. Simulator braking values are not yet
measured hardware values.

### Phase 3 - predefined bay maneuvers, without perception uncertainty

Add a small set of authored bays and connections. Audit stock BT/action
composition first; extend the existing mission node only for the missing
bay-selection, preemption, wait and resume policy.

Drive the policy with explicitly labeled **oracle/test traffic events**.
They test decision logic, not camera capability. Keep these producers
sim-only and outside production launch paths.

**Sim gates:** enter the correct permitted bay before a scripted actor reaches
the conflict area; remain wholly clear while it passes; rejoin without
contact or losing route progress. Test both travel directions, occupied bay,
blocked access, repeated events, second arriving actor, loss of localization,
late detection, occlusion while waiting and no reachable bay.
Where yielding is impossible, report infeasibility/fault; do not count a
stop in the lane as success. Reverse is outside the initial gate.

Use dynamic objects with sensor-visible and collision geometry. A visual-only
Gazebo actor is not enough to validate lidar/depth sensing or contact.

### Phase 4 - monocular camera and terrain/perception integration

Before adopting a model, select/pin a supported combination of Jetson module,
JetPack, ROS distribution, inference runtime, model license and wrapper.
Keep the existing Jazzy/Harmonic simulation baseline while investigating
compatibility; do not silently port the workspace or assume an arbitrary
Isaac ROS release supports Orin Nano and this camera pipeline.

**Compatibility check, 2026-09-09:** NVIDIA's
[Isaac ROS 4.6.0 release notes](https://nvidia-isaac-ros.github.io/releases/index.html#isaac-ros-4-6-0-august-18-2026)
add Jetson Orin and JetPack 7.2 support; the current
[setup requirements](https://nvidia-isaac-ros.github.io/getting_started/index.html#system-requirements)
specify JetPack 7.2 for Jetson and ROS 2 Jazzy. The segmentation repository's
performance table includes Orin Nano Super 8GB. Evaluate this version family
first, checking the exact board and each selected package. Older
JetPack 6 / Isaac ROS 3.x instructions are not grounds to downgrade the
working Jazzy baseline. This is documentation evidence, not a target build
or performance result.

Add calibrated RGB simulation. Keep ideal depth and semantic labels on
separate oracle/scoring topics. The production-like branch must run the
actual selected models on RGB, without receiving perfect simulated depth.
Add delay, dropout, depth-scale bias, invalid edges, lighting/texture changes,
camera pitch/roll and occlusion cases. Compare geometry-only, advisory
semantics and qualified semantic constraints separately.

**Sim gates:** verify topic/frame/time contracts, bounded stale-data response,
no expansion of permitted areas, and no false-free clearing of known hazards.
Measure depth error versus distance, hazardous-ground false-free rate,
traffic recall/false yields and sensor-to-command latency. Freeze scenario
thresholds before tuning. Perfect sensors test integration; model runs on
rendered images test that synthetic domain only, not outdoor accuracy.

Do not make nvblox, visual SLAM or a new 3D terrain mapper prerequisites for
the first bounded detours. Evaluate additional mapping only if it solves a
measured need and accepts the qualified depth/pose inputs on supported hardware.

### Phase 5 - camera-triggered yielding and fault/coverage sweeps

Replace oracle traffic events with events from the selected perception
pipeline. Reuse Phase 3's mission policy; no second traffic navigation stack.
Keep lidar stopping independent of successful classification.

**Sim gates:** reproduce Phase 3 with actual model inputs; sweep arrival
times, speeds, camera visibility, bay spacing and compute latency. Verify
the feasibility bound and full-footprint separation against ground truth.
Include parked traffic, pedestrians/cyclists, crossing/approaching actors,
blocked bays, sensor disagreement and missed detections. Score expected
yield waits separately from unexplained stalls, using scenario truth rather
than trusting a self-reported waiting state.

Maximum admitted traffic speed, minimum detection range, bay geometry,
clearance and resume timing must be explicit scenario parameters. "Low speed"
is not a numeric requirement; stress-test speeds are not supported-operation
claims. Never relax collision/keepout gates to improve completion rate.

### Phase 6 - Jetson integration and field qualification (deferred)

On the actual Jetson, measure the complete pipeline, not isolated model FPS:
Nav2 cycle deadlines, capture-to-constraint age, safety response, CPU/GPU
load, memory pressure, power and thermal throttling under sustained use.
Stock MPPI is not accelerated merely by having a CUDA GPU. Keep its 20 Hz
target, record deadline misses and latency tails, and qualify worst-case
stopping/yield envelopes under perception load. Revalidate RPP if used.

Then perform the original GNSS soak and yaw checks, sensor extrinsics/time
calibration, actual braking and watchdog tests, and supervised closed-area
static/bay/traffic tests. Re-measure camera depth and detection performance
in the intended outdoor conditions. No assumption of public-road readiness.

Operational limits still to freeze: actual camera/FOV and mounting, admitted
robot/traffic speeds, daylight/weather range, allowable slopes/surfaces and
bay spacing/visibility. Scope the first tests to surveyed, firm terrain;
unknown ground, drop-offs and unrestricted road traffic are not validated.

### Phase 7 - reproducibility and soak

Record versioned maps/models/configuration, random seeds, rosbag2 data and
ground-truth scores. Soak with randomized blocks, actors and sensor faults.
Archive oracle and learned-perception results separately. Write ADR-0004 from
measured navigation parity with the teach-driver caveat; do not wait until
hardening to record the architectural decision.

## Immediate next deliverable

**Implement only Phase 2 first:** Nav2-specific ground-truth obstacle scoring,
a standard keepout/speed-mask fixture and the validated reactive stop path,
then R4-N. No Jetson purchase, camera model, custom tracker or field test is
needed for that work. Develop the Phase 4 package/version shortlist in
parallel without changing the working navigation baseline.

Run simulation on an explicit isolated ROS domain (42 was used), preserve
the stale-node check and use only one Gazebo server. If running multiple
worlds later, isolate Gazebo transport as well as ROS.

## Upstream references

The package/version shortlist above links directly to upstream sources.
For reactive stopping, see Nav2's
[Collision Monitor documentation](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/):
it explicitly distinguishes software collision monitoring from hard-real-time
safety certification and documents source timeouts. That page tracks rolling;
confirm parameters and features against the deployed Jazzy release before use.
