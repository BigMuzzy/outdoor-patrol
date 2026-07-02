# GNSS heading wrong in RViz — ANT2 has no signal (KSXT HeadingQual=0)

- **Date:** 2026-07-02
- **Affects:** Unicore UM982 / WTRTK-982 dual-antenna heading, `um982_driver`,
  `outdoor_patrol_loc` (ekf_global `imu0` = `/gnss/heading`)
- **Severity:** gotcha (silent — looked like a fusion/offset bug, was hardware)

## Symptom

The robot's **orientation in RViz was wrong** (pointed a fixed direction
regardless of how it actually faced), even though position was RTK-fixed and
`/gnss/heading` published at a healthy 5 Hz. The fused yaw matched
`/gnss/heading`, so the EKF was tracking it correctly — the *input* was bad.

Tell-tale: `/um982_driver/heading` read **exactly 90.000° ENU** (quaternion
`z=0.70710678…`, i.e. compass heading exactly `0.00°`) with **no noise**. A real
dual-antenna heading jitters ~0.2°; an exact value is a placeholder.

## Root cause

The dual-antenna heading was **not solved**. Decoding the raw `$KSXT` sentence
with the receiver manual (field table in
[docs/](../../../../src/um982_driver/docs/)):

```
$KSXT,…,<hdg>,…, PosQual, HdgQual, #hsolnSVs, #msolnSVs, …
                    3        0         0          24
```
- Field 11 **PosQual = 3** (RTK-fixed position) — ANT1 healthy, 24 sats.
- Field 12 **HdgQual = 0** (heading invalid).
- Field 13 **#hsolnSVs = 0** — the **slave antenna (ANT2) tracked ZERO
  satellites.** (Field 14 `#msolnSVs` = master/ANT1 sats = 24.)

No ANT2 satellites → the ANT1→ANT2 baseline can't be resolved → `HdgQual=0` →
the receiver emits a placeholder heading `0.00°`. `um982_driver` forwarded that
`0.00°` **without checking `HdgQual`**, so a confidently-wrong "East" heading
was fused into `ekf_global` (`imu0`). It's **hardware** (ANT2 signal), not a
yaw-offset or fusion bug.

Note: the manual confirms *"heading is enabled by default for dual-antenna
receivers,"* so it was not a missing-config problem. A prior compass check had
"passed" only because the robot coincidentally faced ~East, matching the
placeholder.

## Fix

- **Hardware:** restore ANT2 signal — reseat/replace the ANT2 coax at both ends,
  confirm ANT2 is a powered active antenna with clear sky view. After reseating,
  `#hsolnSVs` went 0 → 23 and `HdgQual` → 3.
- **Driver robustness (commit):** `um982_driver` now **gates
  `/um982_driver/heading` on `min_heading_quality` (default 1)** — it drops the
  invalid `HdgQual=0` placeholder instead of publishing it, so localization
  falls back to wheel-odom yaw (visibly drifting) rather than pointing
  confidently wrong. Require RTK-fixed heading only with `min_heading_quality:=3`.
- (Optional) `CONFIG HEADING LENGTH 84 5` + `SAVECONFIG` robustifies the rigid
  0.84 m baseline once ANT2 is tracking.

## How to verify

```bash
# Watch ANT2 sat count + heading quality (want ANT2 > ~5, HdgQual 3):
ros2 topic echo /um982_driver/nmea_sentence | grep -m1 -oE 'KSXT[^*]*' | \
  cut -d, -f12,13,14   # -> HdgQual, ANT2_sats, ANT1_sats
```
Healthy: `HdgQual=3`, `ANT2_sats≈ANT1_sats`, and the heading value **jitters**
(real), not a frozen `0.00`.

## Related

- `src/um982_driver/src/unicore_parser.cpp` (KSXT field mapping),
  `um982_driver_node.cpp` `publish_ksxt` (the quality gate)
- Heading offset: `src/outdoor_patrol_loc/config/heading_to_imu.yaml`
- Fusion: `src/outdoor_patrol_loc/config/ekf_global.yaml` (`imu0 = /gnss/heading`)
- [rtk-fix-hard-to-hold-while-moving.md](rtk-fix-hard-to-hold-while-moving.md)
