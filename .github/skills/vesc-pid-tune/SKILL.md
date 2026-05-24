---
name: vesc-pid-tune
description: "Use when the user asks to tune VESC speed-PID gains, run a tuning round, dial in s_pid_kp/ki/kd, analyse step response, or close the iteration loop (capture → analyse → adjust → verify). Per-side semi-automatic loop using scripts/vesc_tune/vesc_tune.py + analyze_step.py + firmware `gains set`. DO NOT USE for full mcconf overhauls, motor detection, hall calibration, or hardware diagnosis."
---

# VESC speed-PID tuning loop (per-side, semi-automatic)

Closed-loop iteration:

1. **Capture** step responses through the existing tune CLI.
2. **Analyse** with [scripts/vesc_tune/analyze_step.py](../../../scripts/vesc_tune/analyze_step.py).
3. **Propose** new gains using the heuristics table below.
4. **Apply** via `gains set` (RAM only) over CAN.
5. **Verify** with a fresh capture.
6. **Report** metrics diff, then **stop** — user reads, decides whether to continue or save.

Each invocation tunes **one side** (`L` or `R`). The right-side drivetrain has known mechanical asymmetry (memory note `mechanical-asymmetry-right`) — do not assume the same gains will satisfy both motors.

## Preconditions (verify before round 1)

- [ ] Wheels chocked / robot on stands. The step amplitudes here spin them.
- [ ] RC TX is on and out of failsafe; tune-mode override is the only path that produces motion.
- [ ] ESP32 reachable: `python3 scripts/vesc_tune/vesc_tune.py --host <IP> status` returns a status line in <2 s.
- [ ] Battery ≥ 38 V (read `vinL`/`vinR` from status).
- [ ] User has confirmed which side: ask once if not stated.

If any precondition fails, **stop and report** — do not silently substitute defaults.

## Setup

1. Ask the user for the ESP32 IP if not already in context. Cache as `$HOST` for the rest of the run.
2. Create a fresh run directory:
   ```bash
   TS=$(date +%Y%m%d_%H%M%S)
   RUN_DIR="runs/skill/${TS}_<side>"
   mkdir -p "$RUN_DIR"
   ```
3. Record the starting gains. There is no read-back yet — instead, snapshot the user's last-stated values (or the mcconf XML) into `$RUN_DIR/gains_initial.txt`. Format: `kp=... ki=... kd=... ramp=...`.

## Capture protocol (one round)

For each round, capture **two** step sizes — a small one (linearity check) and a medium one (the working point):

```bash
python3 scripts/vesc_tune/vesc_tune.py --host "$HOST" step \
    --side <L|R> --e0 0 --e1 1500 --ms 800 --log-hz 100 \
    --out "$RUN_DIR/round${N}_<side>_1500.csv"

python3 scripts/vesc_tune/vesc_tune.py --host "$HOST" step \
    --side <L|R> --e0 0 --e1 3000 --ms 800 --log-hz 100 \
    --out "$RUN_DIR/round${N}_<side>_3000.csv"
```

Add a `--e1 5000` capture **only after** the 3000 run shows OS%≤15 and i_peak ≤ 0.7 × `l_current_max` (currently 25.32 A → cap ≈ 17 A). The right side at 5000 has historically pulled 18.8 A peak; do not push there until it's cleaner.

Between captures the firmware auto-disables after `exp_done`. No explicit `stop` needed.

## Analyse

```bash
scripts/vesc_tune/analyze_step.py "$RUN_DIR/round${N}_<side>_*.csv" \
    --format csv > "$RUN_DIR/round${N}_metrics.csv"
scripts/vesc_tune/analyze_step.py "$RUN_DIR/round${N}_<side>_*.csv"
```

Show the human-readable table to the user. The metrics that matter:

| Metric | Target (tight) | What it tells you |
|--------|----------------|-------------------|
| `rise10_90_ms` | 60–180 ms | speed of response |
| `os_pct`  | ≤ 8 %         | overshoot |
| `us_pct`  | ≤ 3 %         | undershoot on return (matters for braking dynamics) |
| `settle5_ms` | ≤ 300 ms   | time to within ±5 % of setpoint |
| `ss_err_pct` | ≤ 2 %      | steady-state error (proxy for `ki` adequacy) |
| `i_peak_A`   | < 0.7·`l_current_max` | sanity / mechanical health |

## Heuristic adjustment table

Apply **at most two** changes per round. One direction at a time; isolate cause.

| Observed                                  | Adjust                                  | Rationale |
|-------------------------------------------|-----------------------------------------|-----------|
| `rise10_90_ms > 200` and `os_pct < 4`     | `kp *= 1.5`                             | sluggish, headroom for stiffness |
| `os_pct > 12`                             | `kp *= 0.7`                             | too stiff |
| `os_pct > 8` and `rise10_90_ms < 120`     | `kd += 0.0005` (cap at 0.002)           | add damping, keep speed |
| `ss_err_pct > 3`                          | `ki *= 1.3`                             | needs more integral pull |
| `ss_err_pct < 0.5` and oscillation visible | `ki *= 0.7`                             | windup / integrator-driven hunting |
| `i_peak_A` near `l_current_max`           | `ramp *= 0.6` (don't go below 5000)     | electrical headroom |
| 1500 step clean but 3000 step ugly        | leave gains, investigate non-linearity   | likely friction or hall — **stop** |

If two rules apply to opposite parameters (e.g. shrink `kp` AND grow `kd`), apply both — that pair is the canonical "tame overshoot without losing speed" move.

If no rule fires (everything within targets), the round has **converged** — report and ask user whether to save (see *Persistence*).

## Apply gains

Use the existing CLI tunnel. Always include `enable` first; never include `gains save` automatically.

```bash
printf 'enable\ngains set <L|R> kp=%g ki=%g kd=%g ramp=%g\ndisable\n' \
    "$KP" "$KI" "$KD" "$RAMP" \
| python3 scripts/vesc_tune/vesc_tune.py --host "$HOST" repl
```

Note: the repl sends `stop` on EOF — that is harmless after `disable` (no override is active) and serves as belt-and-braces.

Record the new gains in `$RUN_DIR/round${N}_gains.txt` **before** capturing the verification.

## Verify

Repeat the same step sizes as the capture protocol, but suffix the file names `_verify`. Re-run `analyze_step.py`. Produce a side-by-side comparison:

```
metric          before        after       Δ
rise10_90_ms     215           142       -73
os_pct            3.1           7.4      +4.3
settle5_ms       410           260      -150
ss_err_pct        3.8           1.4      -2.4
i_peak_A          4.2           5.1      +0.9
```

Then **stop**. Print:

```
round N complete. metrics within targets: <yes/no>. proceed to round N+1? (reply: continue / stop / save)
```

Wait for the user.

## Persistence

Never call `gains save` without an explicit user instruction. When the user says save:

1. Confirm the side and gain values one more time.
2. Run:
   ```bash
   printf 'enable\ngains save <L|R>\ndisable\n' \
   | python3 scripts/vesc_tune/vesc_tune.py --host "$HOST" repl
   ```
3. Copy the final gains into `$RUN_DIR/gains_final.txt` and append a line to [scripts/vesc_tune/LIVE_TESTS.md](../../../scripts/vesc_tune/LIVE_TESTS.md):
   `YYYY-MM-DD side=<L|R> kp=... ki=... kd=... ramp=... rise=...ms os=...% settle=...ms`.
4. Remind the user the VESC will keep these across power cycles, but the mcconf XML on disk (`configs/vesc/vesc_mcconf_*.xml`) is now stale — they should re-export from VESC Tool when convenient.

## Safety stop conditions

Abort the loop and surface a clear message if any of these are seen during a round:

- `vesc_ok=0` in any `status` reply.
- `i_peak_A > 0.85 × l_current_max` (20.5 A on this hardware).
- Two consecutive rounds where `os_pct` increases by more than 3 percentage points → likely entering instability; revert to last good gains via `gains set`.
- `analyze_step.py` reports `n_samples < 50` for a 800 ms run → logging dropped or transport flaky; do not adjust gains on bad data.

## Anti-patterns

- **Don't** tune `ramp` and PID gains in the same round — you can't tell which fixed what.
- **Don't** chase tiny `ss_err_pct` improvements (<0.5 pp) by raising `ki` — that path leads to windup oscillation on long ramps.
- **Don't** assume L gains transfer to R. Always run R's own loop.
- **Don't** auto-save. The user said so.
