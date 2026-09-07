# Baseline — R3 clean lap, current `route_follower`

Three scored runs of the current stack, frozen as the Phase 1 parity bar.
Each run is `teach r3 r4 r5`, headless, on the same world and through the same
harness Phase 1 uses.

**Machine:** `max-lnx-dev`, Intel i7-8650U @ 1.90 GHz, 8 threads, 15 GB RAM.
Devcontainer, ROS 2 Jazzy, headless (no `GUI=`). The sim held real time: R3
laps took 149–166 s of wall clock for ~100 m of route.
**Commit:** not recorded — this is a git worktree whose `gitdir` lives on the
host (`/home/max/projects/outdoor-patrol/.git/worktrees/...`) and is not
mounted into the container, so `git rev-parse` cannot run here. The tree is
branch `agents/read-nav2-migration-doc` plus the two Phase 1 build fixes
listed in [progress.md](../../progress.md).
**Date:** 2026-09-06

| Run | `cross_track_rms_m` | `cross_track_max_m` | `laps` | `longest_stop_s` | `duration_s` | Result |
|---|---|---|---|---|---|---|
| run1 | 0.0612 | 0.1616 | 0.9986 | 0.89 | 166.4 | PASS |
| run2 | 0.0642 | 0.1726 | 0.9990 | 0.66 | 149.9 | PASS |
| run3 | 0.0681 | 0.1737 | 0.9985 | 0.67 | 148.9 | PASS |
| **mean** | **0.0645** | **0.1693** | **0.9987** | **0.74** | **155.1** | 3/3 PASS |

```
NAV_MAX_RMS = 2 × mean cross_track_rms_m = 0.129
```

That number is now the default in `run_validation.sh` and the worked example
in [phase-1-validation.md](../../phase-1-validation.md) step 3.

The spread is 0.0612–0.0681 m, a range of 0.007 m: the follower is repeatable,
so the parity bar is a measurement rather than an artefact of one lucky lap.
The numbers also land inside the 0.064–0.066 m band that
[issue-8-teach-and-repeat.md](../../../issue-8-teach-and-repeat.md) recorded
over four runs, which is the best available evidence that this machine is not
itself the variable.

## R4 and R5

Recorded for completeness — Phase 1 does not gate on them, because there is no
R4-N yet (see finding 2 in [progress.md](../../progress.md)) and R5-N is
pass/fail rather than a comparison.

| Run | R4 result | R4 min clearance (m) | R5 result | R5 `degraded_cycles` |
|---|---|---|---|---|
| run1 | PASS | 0.240 | PASS | 782 |
| run2 | PASS | 0.245 | PASS | 783 |
| run3 | PASS | 0.188 | PASS | 787 |

R4's minimum clearance is at `barrier_1` in all three runs, against an ideal
of 0.298 m and a gate of 0.15 m. Run 3's 0.188 m is the tightest of the nine
barrier passes and is the number a Phase 2 R4-N has to beat — the margin over
the gate is only 0.038 m.

R5's `degraded_cycles` of ~782 is far above the R5-N gate of 20, but the two
are not comparable: the follower publishes status at 20 Hz for the whole
degraded dwell, whereas R5-N counts mission-level cycles. Do not read a much
smaller Nav2 number as a regression.

## Notes

- All nine scored runs passed first time; nothing was re-run, re-taught or
  discarded, so the mean above is over the full set, not a selected subset.
- R5's `longest_stop_s` is ~45.7 s in every run. That is the point of R5 — the
  robot stops on a degraded fix and stays stopped — not a fault. Only R3's
  `longest_stop_s` is gated, against 3 s.
- The teach pass was re-driven at the start of each run rather than shared, so
  route-recording variation is inside the spread quoted above rather than
  factored out of it.
