# Phase 0 — freeze the baseline

**Deliverable:** three scored runs of the *existing* follower, and one number
carried into every later phase.

No code is written in this phase. Nothing in this phase changes the robot.

Runnable instructions: [phase-0-validation.md](./phase-0-validation.md).

## Why it exists

Phase 1 replaces the thing that drives the robot. "Nav2 works" is not a result;
"Nav2 tracks the lane at least as well as what it replaced" is. That comparison
needs a baseline measured on **this** machine, because the harness's numbers
move with the real-time factor the host can sustain.

[issue-8-teach-and-repeat.md](../issue-8-teach-and-repeat.md) reports R3
cross-track RMS of 0.064–0.066 m over four runs. That is the expected answer,
not the baseline. Measure your own.

## What to run

Three repetitions of the existing suite, each into its own directory:

```bash
src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/baseline/run1 teach r3 r4 r5
src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/baseline/run2 teach r3 r4 r5
src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/baseline/run3 teach r3 r4 r5
```

Each repetition re-teaches the route. That is deliberate: the teach pass is a
source of run-to-run variation, and a baseline that hides it would understate
the spread that Phase 1 has to beat.

Roughly 25 minutes per repetition headless, so about 75 minutes total. Run it
headless — `GUI=1` costs real-time factor, and the pass/fail numbers have to
come from the headless configuration that Phase 1 will also use.

## The parity bar

```
NAV_MAX_RMS = 2 × mean(R3 cross_track_rms_m over run1, run2, run3)
```

Placeholder in `run_validation.sh` is **0.128** — twice the 0.064 m from
issue-8. Overwrite it with the measured number:

```bash
NAV_MAX_RMS=<your number> src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/val r3n
```

Why 2× and not 1×: Nav2 is not a pure-pursuit controller tracking a spline. It
plans through a costmap and MPPI trades tracking against clearance and control
effort. A stack that tracks within twice the baseline while gaining reverse
motion, recoveries and free obstacle avoidance is a win; one that needs 5× is
not, and the 2× bar is what makes that a decision instead of an argument.

## What to record

Commit these, per repetition:

1. `score_R3.json`, `score_R4.json`, `score_R5.json`
2. A row in `runs/baseline/baseline.md` with the five headline numbers

Do **not** commit the bags. See [runs/baseline/README.md](./runs/baseline/README.md).

The five numbers that matter, all from `score_R3.json`:

| Field | Why it is the bar |
|---|---|
| `cross_track_rms_m` | the parity bar itself |
| `cross_track_max_m` | catches a stack that is accurate on average and wild at corners |
| `laps` | completeness; anything below 0.98 is a failed run, not a slow one |
| `longest_stop_s` | Nav2 stops at goals in ways the follower never did |
| `duration_s` | a Nav2 lap that takes twice as long is a different mission |

## Field prerequisite

Phase 0 is a simulation phase, but the field gates it depends on come from
[field-validation-alley.md](../field-validation-alley.md), and only the first
two are needed before Phase 1:

- **Phase 1 — GNSS soak (15 min).** The go/no-go. Establishes what horizontal
  sigma the site actually delivers, which is what `sigma_slow_m` and
  `sigma_stop_m` are set against.
- **Phase 2 — Verify heading (20 min).** `yaw_offset`. A wrong heading offset
  produces a stack that tracks perfectly in sim and drives into the fence
  outdoors.

Everything after those (teach pass, first autonomous run, obstacle) waits for
Phase 1 of this plan to pass in simulation. Do not take an unvalidated Nav2
stack outdoors.

## Done when

- Three `score_R3.json` files exist and all three passed.
- `runs/baseline/baseline.md` has three rows and a mean.
- `NAV_MAX_RMS` is written into
  [phase-1-validation.md](./phase-1-validation.md) step 3.
