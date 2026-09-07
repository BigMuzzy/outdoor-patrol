# Phase 0 validation

Run this first:

```bash
src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/baseline/run1 teach r3 r4 r5
```

~25 minutes, headless. Do not pass `GUI=1` — rendering costs real-time factor
and the numbers have to come from the same configuration Phase 1 uses.

## Steps

1. Build, if you have not since the last pull. ~3 minutes.

   ```bash
   colcon build --merge-install && source install/setup.bash
   ```

2. Run the suite three times, each into its own directory. ~75 minutes total.

   ```bash
   src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/baseline/run1 teach r3 r4 r5
   src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/baseline/run2 teach r3 r4 r5
   src/outdoor_patrol_sim/scripts/run_validation.sh /tmp/baseline/run3 teach r3 r4 r5
   ```

3. Read the three R3 numbers. ~1 minute.

   ```bash
   for n in 1 2 3; do
     python3 -c "import json,sys; d=json.load(open(sys.argv[1])); \
       print('%.4f %.4f %.3f %.1f' % (d['cross_track_rms_m'], \
       d['cross_track_max_m'], d['laps'], d['longest_stop_s']))" \
       /tmp/baseline/run$n/score_R3.json
   done
   ```

4. Copy the artefacts into the repo. ~2 minutes.

   ```bash
   for n in 1 2 3; do
     mkdir -p doc/eng/plans/nav2-migration/runs/baseline/run$n
     cp /tmp/baseline/run$n/score_*.json \
        doc/eng/plans/nav2-migration/runs/baseline/run$n/
   done
   ```

5. Write the mean into
   `doc/eng/plans/nav2-migration/runs/baseline/baseline.md`, and double it.
   That doubled number is `NAV_MAX_RMS`. ~5 minutes.

## Pass criteria

All three repetitions must end in `validation complete: PASS`. If one fails,
that is a finding about the current stack, not about Nav2 — fix or explain it
before starting Phase 1, because the baseline is meaningless otherwise.

Expected, from issue-8 over four runs:

| Metric | Expected |
|---|---|
| `cross_track_rms_m` (R3) | 0.064–0.066 m |
| `cross_track_max_m` (R3) | 0.161–0.173 m |
| `laps` | ≥ 0.98 |
| `longest_stop_s` (R3) | < 3 s |

## If it fails

**`stack nodes already on the ROS graph`** — a previous run left something
behind. Cause: a process escaped the group kill. Fix:

```bash
pkill -f 'gz sim'; pkill -f parameter_bridge; sleep 3; ros2 node list
```

**`/odometry/global never appeared`** — Gazebo did not reach the point of
publishing `/clock` within `READY_TIMEOUT`. Cause is usually rendering: check
`/tmp/baseline/run1/sim_teach.log` for an EGL error and, if you see one, re-run
with `software_rendering:=true`.

**`route_odometry_global.yaml missing`** — you ran `r3` without `teach` in a
fresh directory. Fix: include `teach` in the run list.

## Next

Run step 1 now: `colcon build --merge-install`.
