# 0002 — IMU: Inertial Labs KERNEL instead of canonical Xsens MTi-30

- **Date:** 2026-06-27 (recorded; `imu_driver` landed earlier, commit `59b93bb`)
- **Status:** Accepted (provisional — confirm at M2 EKF fusion + M5 field validation)
- **Affects:** `src/imu_driver`, M2 (EKF yaw/accel fusion), M5 / [ADR-012](../../../src/robot-research/notes/outdoor-patrol/recipe/decisions.md) (heading)
- **Supersedes for the as-built robot:**
  recipe **HW-ADR-004** and the IMU rows of
  [`hardware/components-index.md`](../../../src/robot-research/notes/outdoor-patrol/hardware/components-index.md)

## Context

The research recipe selects the **Xsens MTi-30** 9-axis AHRS as the
canonical navigation IMU
([HW-ADR-004](../../../src/robot-research/notes/outdoor-patrol/hardware/decisions.md)),
interfaced via the mature `xsens_mti_ros2_driver`, with the magnetometer
disabled by default (recipe heading derived from LIO + GNSS).

The as-built robot instead uses an **Inertial Labs KERNEL-family IMU**,
for which we wrote an in-house ROS 2 driver (`imu_driver`). This record
captures the substitution and its implications.

## Decision

Use the Inertial Labs KERNEL as the navigation IMU, read by the in-house
`imu_driver`.

Verifiable engineering facts behind the substitution:

1. **Hardware on hand + an in-house driver already exists.** `imu_driver`
   is built and unit-tested; it speaks the KERNEL binary wire protocol
   (`0xAA 0x55` framing, *KERNEL ICD rev 1.39*), streams the device
   **Calibrated HR Data**, publishes `sensor_msgs/Imu` +
   `sensor_msgs/Temperature`, and decodes the Unit Status Word into
   `/diagnostics`. It is a lifecycle node and only reads the device.
2. **Supplies the M2 EKF inputs.** The device provides the gyro
   `angular_velocity.z` (`vyaw`) and linear acceleration the
   [ADR-001](../../../src/robot-research/notes/outdoor-patrol/recipe/decisions.md)
   EKF needs. High device rate (~2 kHz) is decimated to ~100 Hz by block
   averaging (`publish_every_n`), reducing noise ~√N.

> The procurement/selection rationale (cost, availability, vendor
> relationship vs. the Xsens line) is **not fully reconstructed here** —
> confirm with the team and fold into the final HW-ADR-004 amendment.

## Consequences

- **No magnetometer → relative yaw only.** The KERNEL Calibrated HR
  heading is relative (the driver README documents this and defaults
  `orientation_covariance` yaw to a large value). The robot therefore has
  **no IMU-derived absolute heading**. This is the same end-state the
  recipe intended (MTi-30 mag disabled), but it makes the absolute-heading
  source an explicit, load-bearing decision:
  - absolute yaw comes from the **UM982 dual-antenna GNSS heading**
    ([eng ADR-0001](0001-gnss-receiver-um982-vs-zed-f9p.md),
    [ADR-012](../../../src/robot-research/notes/outdoor-patrol/recipe/decisions.md))
    and, once it lands, **LIO yaw**;
  - the arbitration/handoff between GNSS-heading, LIO yaw, and
    gyro-integrated yaw is now tracked as recipe **OQ-026**.
- **Sensor-frame care.** Data is published in the IMU's own axes
  (X-right, Y-forward, Z-up), **not** REP-103 `base_link`. A static
  `base_link → imu_link` transform must encode the physical mount; an
  `imu_link` module entry is owed in
  `src/outdoor_patrol_bringup/config/chassis.yaml` (currently only
  `gnss_link` is defined).
- **Recipe drift.** HW-ADR-004 and the components index still name the
  MTi-30; they should be amended to record the KERNEL as the accepted
  substitution (or this record promoted into the recipe submodule).
- **Validation owed at M2:** re-run the spin/square drift tests with the
  KERNEL fused (`imu0`) and confirm the yaw-rate quality meets the M2
  acceptance bound. **Heading consumption is not wired yet** — `imu_driver`
  publishes but no EKF consumes it (M2 not started).

## Alternatives considered

- **Xsens MTi-30 (canonical).** Mature `xsens_mti_ros2_driver`, factory
  pre-calibration, documented in-run bias stability. Kept as the
  documented fallback if KERNEL drift disappoints at M2.
- **Reuse the GNSS dual-antenna heading as the only yaw source.** Rejected
  as insufficient on its own: GNSS heading degrades at short baseline /
  low speed and during GNSS dropouts, so an inertial yaw-rate source is
  still required for the dead-reckoning fill (this is exactly the OQ-026
  arbitration problem).

## Related

- `src/imu_driver/README.md`, `src/imu_driver/docs/` (KERNEL ICD rev 1.39)
- Recipe: HW-ADR-004, ADR-001, ADR-012, OQ-026, implementation-plan M2/M5
- [eng ADR-0001](0001-gnss-receiver-um982-vs-zed-f9p.md) (GNSS dual-antenna heading)
