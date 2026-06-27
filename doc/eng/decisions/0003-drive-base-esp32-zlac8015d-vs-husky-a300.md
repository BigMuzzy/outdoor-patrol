# 0003 — Drive base: custom ESP32-S3 diff-drive + ZLAC8015D instead of canonical Husky A300

- **Date:** 2026-06-27 (recorded; firmware migration is esp32 ADR-0010, dated 2026-06-06)
- **Status:** Accepted
- **Affects:** `src/esp32-s3-uros-controller`, `src/outdoor_patrol_bringup`
  (URDF + `chassis.yaml`), M0–M1 (teleop + wheel odometry), [ADR-004](../../../src/robot-research/notes/outdoor-patrol/recipe/decisions.md)
  (Smac motion-primitive set, OQ-012)
- **Supersedes for the as-built robot:**
  recipe **HW-ADR-007** (Clearpath Husky A300) and the chassis/motor rows of
  [`hardware/components-index.md`](../../../src/robot-research/notes/outdoor-patrol/hardware/components-index.md)

## Context

The research hardware track selects the **Clearpath Husky A300**
([HW-ADR-007](../../../src/robot-research/notes/outdoor-patrol/hardware/decisions.md)):
a 4-wheel **skid-steer** platform with an integrated motor stack and the
`clearpath_common` / `husky_robot` Jazzy packages, collapsing the chassis
(HQ-009) and motor-driver (HQ-011) questions into one vendor stack.

The as-built robot is instead a **custom 2-wheel differential-drive
chassis** driven by an **ESP32-S3 micro-ROS controller**
(`esp32-s3-uros-controller`). Its motor stack migrated from dual VESC
controllers to a **ZLAC8015D V4** dual-channel hub-servo drive speaking
**CANopen / CiA 402**, with two **ZLLG65ASM250 V3.0** 6.5″ direct-drive
hub motors. The firmware-level decision and object-dictionary mapping are
recorded in the controller submodule's
[ADR-0010](../../../src/esp32-s3-uros-controller/docs/adr/0010-zlac8015d-canopen-migration.md)
(which supersedes its ADR-0003 VESC protocol for the current build; the
VESC backend remains selectable for legacy units). This record captures
the **superproject-level** drift versus the recipe and its planning
consequences.

## Decision

Use the custom ESP32-S3 differential-drive chassis with the ZLAC8015D /
ZLLG65ASM250 motor stack as the as-built drive base. The robot presents
the standard contract the recipe assumes: it consumes
`geometry_msgs/Twist` on `/cmd_vel` and publishes wheel odometry
(`/odom`).

Rationale:

1. **Matches the implementation-plan's locked assumption.** The recipe's
   own [implementation-plan](../../../src/robot-research/notes/outdoor-patrol/recipe/implementation-plan.md)
   locks *"diff-drive, accepts `geometry_msgs/Twist` on `/cmd_vel`"* at the
   start — the as-built 2-wheel diff-drive **is** that assumption. The
   divergence is specifically against HW-ADR-007's *hardware pick*, not
   against the software plan.
2. **Cost and control.** A custom chassis + commodity CANopen hub-drive is
   materially cheaper than the integrated Husky A300, and an in-house
   firmware HAL gives direct control over the odometry, calibration, and
   stop-latency behaviour (see M1 calibration work: stop-lead compensation,
   covariance population) that a sealed vendor stack would not expose.
3. **Hardware-agnostic HAL preserved.** The firmware keeps a backend-select
   HAL (`MOTOR_DRIVER_BACKEND`): ZLAC8015D default, VESC legacy — so the
   motor-controller choice is not load-bearing on the ROS-side contract.

Key geometry (single source of truth: `chassis.yaml` ↔ firmware
`diff_drive.h`): wheel Ø 170 mm (radius 0.08534 m), track 0.52123 m,
encoder 16384 counts/rev.

## Consequences

- **Kinematic-class divergence feeds [ADR-004](../../../src/robot-research/notes/outdoor-patrol/recipe/decisions.md) / OQ-012.**
  HW-ADR-007 assumed **4-wheel skid-steer**; as-built is **2-wheel
  differential drive**. Both are non-Ackermann and share the same Nav2
  planner family (Smac Hybrid-A\* with a diff/omni motion model), so the
  planner *choice* is unaffected — but the **motion-primitive set and the
  controller's kinematic limits must be configured for diff-drive**, not
  skid-steer. Record this when OQ-012 is closed.
- **No vendor Nav2 packages.** `clearpath_common` / `husky_robot` are not
  used; bringup, URDF, and controller config are all in-house
  (`outdoor_patrol_bringup`, `outdoor_patrol_loc`).
- **No integrated battery/charge telemetry.** HW-ADR-007 would have
  supplied stock 24 V/24 Ah Li-ion + vendor charge state. The custom base
  must provide its own SoC/charge signals for the M11
  `BatteryLow → ReturnAndDock` branch ([ADR-010](../../../src/robot-research/notes/outdoor-patrol/recipe/decisions.md));
  not yet present.
- **Recipe drift.** HW-ADR-007 and the components index still name the
  Husky A300; amend them to record the custom base as the accepted
  substitution (or promote this record into the recipe submodule).
- **Weatherproofing owed.** The Husky A300 is IP44 stock; the custom base
  has no stated IP rating. Enclosure/sealing for the [ADR-009](../../../src/robot-research/notes/outdoor-patrol/recipe/decisions.md)
  weather profile is a hardware-track item.

## Alternatives considered

- **Clearpath Husky A300 (canonical).** Integrated, IP44, mature ROS 2
  stack, removes drivetrain engineering. Rejected for the as-built robot
  on cost and firmware-control grounds; remains the reference for a
  productionized v1 if the custom base does not weatherproof/scale.
- **Keep dual VESC** (esp32 ADR-0003). Superseded by ADR-0010 for the
  current build; retained as a selectable legacy backend.
- **AgileX Scout 2.0** (recipe v1.5 cost-down). Still a skid-steer vendor
  stack; not chosen for the bring-up robot.

## Related

- `src/esp32-s3-uros-controller/docs/adr/0010-zlac8015d-canopen-migration.md`
  (motor HAL + CiA-402 mapping), `…/adr/0003-vesc-can-protocol.md`,
  `…/adr/0004-diff-drive-kinematics.md`, `…/adr/0005-odometry-computation.md`
- `src/outdoor_patrol_bringup/config/chassis.yaml`,
  `src/outdoor_patrol_bringup/urdf/outdoor_patrol.urdf.xacro`
- Recipe: HW-ADR-007, ADR-004 (OQ-012), implementation-plan M0–M1
