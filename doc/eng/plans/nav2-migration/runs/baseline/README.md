# Baseline runs

Phase 0 artefacts: three repetitions of the existing validation suite against
the **current** `route_follower`, before Nav2 exists in the run path.

## What goes in here

```
runs/baseline/
  README.md      this file
  baseline.md    one row per repetition + the mean and the parity bar
  run1/score_R3.json  score_R4.json  score_R5.json
  run2/...
  run3/...
```

## What does not

**The bags.** A single `bag_r3` is 60–200 MB of mcap and there are three of
them per repetition. They are reproducible from the harness, and git is not a
bag store. Keep them under `/tmp/baseline/` until Phase 1 passes, then delete
them.

If a run fails in a way the JSON does not explain, keep **that one bag**
outside the repo and link to it from `baseline.md` — a path and a machine name
are enough.

## Why three repetitions

Each one re-teaches the route, so run-to-run spread includes the teach pass.
issue-8 saw R3 RMS range 0.064–0.066 m over four runs; if your three land in a
much wider band, the parity bar is being set by variance rather than by the
controller and that is worth knowing **before** Nav2 is measured against it.

## Regenerating

```bash
src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/baseline/run1 teach r3 r4 r5
```

~25 minutes per repetition, headless. Full instructions:
[../../phase-0-validation.md](../../phase-0-validation.md).
