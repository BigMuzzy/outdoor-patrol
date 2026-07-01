# RTK integer fix is hard to hold while driving (multipath, not geometry)

- **Date:** 2026-07-01
- **Affects:** Unicore UM982 RTK GNSS + PointOne "True RTK" VRS (NTRIP `AUTO`),
  `outdoor_patrol_loc` dual-EKF; robot antenna ANT1 mounted low (~0.33 m) on
  the chassis
- **Severity:** gotcha (expected RTK behavior — *not* a bug)

## Symptom

Field test: **RTK-fixed (~1.3 cm, GGA quality 4)** reliably when **parked in the
open**, but while **driving at ~1–2 m/s** the solution dropped to **RTK-float
(GGA quality 5, σ ≈ 0.2–0.4 m)** and would not re-latch integers for minutes.

Crucially, the *other* health indicators were all good at the same time:

```
RTK-FLOAT  sats=25  age=0.8s  east_sigma=0.24m   # HDOP 0.6–0.8, ~100 m VRS baseline
```

25–28 satellites, HDOP 0.6–0.8, correction age <1.5 s, short (~100 m) VRS
baseline — yet stuck in float.

## Root cause

Integer ambiguity resolution (AR) needs a few seconds of **clean, continuous
carrier-phase** tracking. When satellite geometry (DOP), satellite count, and
corrections (age/baseline) are all good **but you still can't hold a fix, the
limiter is carrier-phase continuity — i.e. multipath and cycle slips**, not the
sky count or the caster.

The robot's antenna sits **low and amid the chassis/electronics**, a
multipath-rich spot (ground bounce + reflections off the robot's own
structure). Driving through/near clutter (fences, trees, buildings, the robot
body itself) causes cycle slips that reset AR back to float. Parked in the
open the phase stays clean, so it fixes and holds.

This matches textbook RTK: every accuracy tier we measured is on the numbers,
so the receiver is behaving correctly.

| Mode | Typical horizontal | Measured 2026-07-01 |
|---|---|---|
| RTK fixed | 1–2 cm (+1 ppm × baseline) | ~1.3 cm |
| RTK float | 0.2–0.5 m | 0.2–0.4 m |
| DGPS/SBAS | 0.3–1 m | ~0.4 m |
| Standalone | 1–3 m | ~1.1 m |

### Speed is not the limiter

RTK works to highway speeds; 1–2 m/s is kinematically trivial. Two notes:

- **Correction *age* ≠ position latency.** At age ~1 s and 2 m/s the base data
  is "~1 s old," but the rover extrapolates with its own carrier phase, so the
  *output* stays cm-accurate and low-latency.
- What actually governs fix-and-hold at 1–2 m/s is the **environment**, not the
  velocity:

| Environment @ 1–2 m/s | Realistic expectation |
|---|---|
| Open sky, good antenna + ground plane, high mount | Sustained RTK-fixed 1–3 cm, holds through motion |
| Partial sky / near buildings, trees, fences | Mix of fixed ↔ float; re-fix in seconds when clear (what we saw) |
| Canopy / urban canyon | Mostly float or worse, occasional loss |

## Fix

Not a software bug — mitigations, in order of impact:

1. **Antenna placement is the #1 lever.** A multi-band survey/helical antenna on
   a **ground plane**, mounted **high and clear** of the robot structure, is the
   single biggest improvement for holding fix while moving. The current low ANT1
   position is the prime suspect for the float-while-moving behavior.
2. **The dual-EKF is designed to tolerate float/dropouts.** The local EKF (wheel
   odom + IMU + dual-antenna heading) dead-reckons through float and short GNSS
   gaps, so `/odometry/global` stays smooth even when RTK degrades. Intermittent
   float while driving is *tolerated by design*, not a failure.
3. **Keep corrections continuous.** NTRIP stall/outage auto-recovery landed in
   `ntrip_client` (stall watchdog + bounded connect); keep correction age <2 s.

## How to verify

- **Diagnostic rule:** good DOP + many sats + fresh corrections but persistent
  **float ⇒ multipath / antenna environment**, not geometry or the caster.
  (Conversely, high age or few sats points at corrections/sky.)
- Live check while driving:
  ```bash
  # fix type + east sigma once/sec
  # RTK-FIXED q=4 (~cm) vs RTK-FLOAT q=5 (dm); see /um982_driver/fix status + cov
  ```
- Expect open-sky parked → fixed in seconds; driving near clutter → float with
  seconds-scale re-fix in the clear.

## Related

- Heading offset verification: `src/outdoor_patrol_loc/config/heading_to_imu.yaml`
- NTRIP auto-recovery: commit `feat(ntrip_client): stall watchdog + bounded connect`
- Dual-EKF GNSS localization: ADR-012 (`src/robot-research/notes/outdoor-patrol/recipe/`)
- [deployment/pi-container-races-wifi-at-boot.md](../deployment/pi-container-races-wifi-at-boot.md)
