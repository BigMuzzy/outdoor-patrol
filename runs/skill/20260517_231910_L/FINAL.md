# Final bench-tuned gains (flashed both sides 2026-05-17)

| param | value | notes |
|---|---|---|
| s_pid_kp | 0.002 | |
| s_pid_ki | 0.02 | |
| s_pid_kd | 0.001 | |
| s_pid_ramp_erpms_s | 5000 | |
| s_pid_min_erpm | 300 | deadband, suppresses near-zero integrator/brake fight |

Source: round 3 (LEFT). Applied identically to RIGHT (no separate tuning round on R).

## Bench performance (LEFT, free-spinning wheels)

| step (ERPM) | OS% | settle | ss_std |
|---|---:|---:|---:|
| 1500 | 45% | 410 ms | ±71 |
| 3000 | 16% | 390 ms | - |
| 500 (op) | 57% | 789 ms | ±286 |
| 1000 (op) | 54% | 790 ms | ±496 (US 101%) |

## Why we stopped
Bench (free-spinning wheels) has no load to absorb overshoot. Op-range targets (500/1000 ERPM)
show heavy oscillation on bench but should behave very differently under real road load.
Further bench tuning would chase artifacts. Next step is field validation on actual robot
at 0.5-1.0 m/s; tune further only if real-load tracking is poor.

## Field-validation procedure
1. Power-cycle both VESCs to confirm flash persistence.
2. Drive robot at 0.5 m/s (~420 ERPM) steady, then 1.0 m/s (~835 ERPM) steady.
3. Capture wheel ERPM telemetry, compare to commanded.
4. If steady-state tracking std > ~5% of target, or step response on real load shows >30% OS,
   re-tune at that point (probably reduce ki to ~0.01).

## Flash persistence verified (2026-05-20)

After full power-cycle of both VESCs:

| side | tgt | OS% | settle | ss_std | rise90 | i_pk |
|---|---:|---:|---:|---:|---:|---:|
| L | 500  | 35% | 780ms | ±82  | 240ms | 3.5A |
| L | 1500 | 26% | 730ms | ±118 | 240ms | 7.8A |
| R | 500  | 40% | 790ms | ±150 | 261ms | 4.2A |
| R | 1500 | 22% | 800ms | ±64  | 260ms | 8.0A |

**This run was performed on the ground under realistic load** (not on the
bench). Tight ss_std and absence of factory-baseline ringing confirm r3
gains + db=300 survived the power-cycle on both VESCs.

Field-validation reading:
- Steady-state tracking is good: ss_std ≤ ±150 ERPM at both 500 and 1500
  targets — adequate for outdoor patrol.
- Higher i_pk (7.8–8.0 A at 1500) and slower rise90 (~240–260 ms) are real
  rolling load (wheels driving robot mass), not gain issues.
- R shows the known asymmetry: tighter at 1500 (±64) but bouncier at 500
  (US 93%, ±150) than L. Same gains aren't optimal for both — address only
  if patrol motion exposes it.
- US% at 500 is a deadband-transition artifact and won't appear during
  continuous patrol motion (no instant stops to zero).

**Tuning declared complete.** Re-tune only if observed patrol behavior
shows sustained tracking errors or surface-dependent instability.

## Automated retune attempt (2026-05-20)

Built `scripts/vesc_tune/auto_tune.py` (noise-aware coordinate descent
with N=3 averaging) and ran two passes on LEFT under load. Conclusion:
the gain landscape around r3 is **flat within measurement noise**.
J values cluster 18–23 across all explored gains; verify runs of the
same gains swing by ~10 J. No statistically meaningful improvement
available with current measurement precision.

**r3 retained in flash as the final tune.** Further automated tuning
would need higher-quality measurements (longer steps, controlled
surface, more reps), not different algorithms.
