# 0001 — GNSS receiver: UM982 (dual-antenna) instead of canonical ZED-F9P

- **Date:** 2026-06-06
- **Status:** Accepted (provisional — confirm at M5 field validation)
- **Affects:** `src/um982_driver`, `src/ntrip_client`, M2 (EKF yaw), M5 (GNSS fusion)
- **Supersedes for the as-built robot:**
  recipe **HW-ADR-003** and the GNSS rows of
  [`hardware/components-index.md`](../../../src/robot-research/notes/outdoor-patrol/hardware/components-index.md)

## Context

The research recipe selects the single-antenna **u-blox ZED-F9P** as the
canonical RTK receiver
([implementation-plan M5](../../../src/robot-research/notes/outdoor-patrol/recipe/implementation-plan.md),
[bom-summary](../../../src/robot-research/notes/outdoor-patrol/hardware/bom-summary.md)).
Dual-antenna heading receivers (e.g. Septentrio mosaic-H) are explicitly
marked *"niche — skip for canonical recipe"* in the components index.

The as-built robot instead uses the **Unicore UM982 / WitMotion
WTRTK-982**, a low-cost **dual-antenna** RTK module, and we wrote an
in-house driver (`um982_driver`) plus a generic correction source
(`ntrip_client`). This record captures why and what it implies.

## Decision

Use the UM982 as the primary GNSS receiver for the patrol robot.

Rationale:

1. **Absolute heading from the GNSS baseline.** The dual-antenna UM982
   reports yaw directly (`~/heading`, KSXT). This gives the EKF an
   absolute heading observation that the single-antenna ZED-F9P cannot,
   reducing dependence on magnetometer calibration for yaw observability
   — directly useful at **M2 (IMU + EKF)** and **M5 (GNSS fusion)**.
2. **Satisfies the ADR-001 gating contract.** The receiver emits
   per-fix RTK status (fix/float/DGPS/single) and HDOP in its NMEA/Unicore
   output, which is what the `confidence_gate`
   ([ADR-001](../../../src/robot-research/notes/outdoor-patrol/recipe/decisions.md))
   needs to inflate covariance on degraded fixes.
3. **Cost.** The WTRTK-982 packaging is materially cheaper than the
   canonical ZED-F9P + helical-antenna line item in the BOM.
4. **Correction-source agnostic.** Corrections arrive on `rtcm/in`
   (`rtcm_msgs/Message`), so NTRIP, raw-TCP, or serial-radio bases all
   work via `ntrip_client` without driver changes.

## Consequences

- **Recipe drift.** HW-ADR-003 and the components index still name the
  ZED-F9P. They should be amended to record the UM982 as the accepted
  substitution (or this record promoted into the recipe submodule).
- **Mechanical:** dual-antenna requires two antennas at a fixed, known
  baseline on the chassis; the baseline length and antenna lever-arm feed
  the driver's `antenna_*` params and the heading calibration.
- **Validation owed at M5:** confirm the UM982's RTK fix-rate and
  fix-status granularity meet the ADR-001 gating quality in the actual
  deployment environment (open-questions
  [OQ-007 / HQ-004](../../../src/robot-research/notes/outdoor-patrol/hardware/questions.md)).
  The M5 GNSS-degraded walk-block test is the acceptance gate.
- **Heading consumption is not wired yet.** `~/heading` is published but
  no EKF consumes it (M2 not started). Treat the heading path as
  unvalidated until M2/M5.

## Alternatives considered

- **u-blox ZED-F9P (canonical).** Mature ecosystem and existing ROS
  drivers, but single-antenna → no GNSS heading; would need a separate
  heading source. Kept as the documented fallback if UM982 RTK
  performance disappoints at M5.
- **Septentrio mosaic-H.** Dual-antenna heading with better multipath
  rejection, but ~5–6× the cost; revisit only for dense-canopy sites.

## Related

- `src/um982_driver/README.md`, `src/ntrip_client/README.md`
- `src/um982_driver/launch/gnss_rtk.launch.py` (combined rover + NTRIP bringup)
- Recipe: HW-ADR-003, ADR-001, implementation-plan M5
