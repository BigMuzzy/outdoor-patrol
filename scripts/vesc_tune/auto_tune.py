#!/usr/bin/env python3
"""Automatic VESC speed-PID tuner (noise-aware coordinate descent).

Iterates over (kp, ki, kd) on one side, applying gains via the existing
tune CLI tunnel, capturing step responses at 500 and 1500 ERPM, and
scoring with a scalar objective. Search is bounded for safety; outright
failures (excess current, lost tracking) return a sentinel cost so the
algorithm rejects them.

Example
-------
    scripts/vesc_tune/auto_tune.py --host 192.168.55.28 --side L \\
        --kp 0.002 --ki 0.02 --kd 0.001 \\
        --out-dir runs/auto/$(date +%Y%m%d_%H%M%S)_L
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from pathlib import Path

# Import metrics() from sibling analyze_step.py
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from analyze_step import metrics as analyze_metrics  # noqa: E402

VESC_TUNE = HERE / "vesc_tune.py"

# Safety bounds
KP_MIN, KP_MAX = 0.0005, 0.005
KI_MIN, KI_MAX = 0.005, 0.040
KD_MIN, KD_MAX = 0.0,    0.003

I_PEAK_LIMIT  = 15.0   # A, hard reject
SS_ERR_LIMIT  = 0.50   # 50 % off target → lost tracking, hard reject
               # (plant is non-linear; baseline can read >30% off)
OS_LIMIT      = 100.0  # % overshoot, hard reject

STEP_TARGETS  = (500, 1500)
STEP_MS       = 800
LOG_HZ        = 100
RECOVER_S     = 1.5    # idle between captures so motor stops fully
N_REPEATS     = 3      # captures per (gains, step) for noise averaging
CAPTURE_RETRY = 2      # retries per capture on TCP/transport failure


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def check_vesc_ok(host: str) -> bool:
    """Return True if firmware reports vesc_ok=1."""
    try:
        r = subprocess.run(
            ["python3", str(VESC_TUNE), "--host", host, "status"],
            capture_output=True, text=True, timeout=5,
        )
        return "vesc_ok=1" in r.stdout
    except Exception:
        return False


def apply_gains(host: str, side: str, kp: float, ki: float, kd: float) -> bool:
    """Send `gains set <side> kp=.. ki=.. kd=..` via repl. Return True on success."""
    cmd = (
        f"enable\n"
        f"gains set {side} kp={kp:.6g} ki={ki:.6g} kd={kd:.6g}\n"
        f"disable\n"
    )
    r = subprocess.run(
        ["python3", str(VESC_TUNE), "--host", host, "repl"],
        input=cmd, capture_output=True, text=True, timeout=10,
    )
    return r.returncode == 0 and "ERR" not in r.stdout


def capture_step(host: str, side: str, e1: int, out_csv: Path) -> bool:
    """Run one step capture, with retry on transport failure."""
    for attempt in range(CAPTURE_RETRY + 1):
        if attempt > 0:
            time.sleep(1.0)
            if not check_vesc_ok(host):
                return False
        r = subprocess.run(
            ["python3", str(VESC_TUNE), "--host", host, "step",
             "--side", side, "--e0", "0", "--e1", str(e1),
             "--ms", str(STEP_MS), "--log-hz", str(LOG_HZ),
             "--out", str(out_csv)],
            capture_output=True, text=True, timeout=20,
        )
        if r.returncode == 0 and out_csv.exists() and out_csv.stat().st_size > 100:
            return True
        # treat as transient; retry
    return False


def avg_metrics(ms: list[dict]) -> dict:
    """Average numeric fields across repeated captures of the same step."""
    keys_avg = ("overshoot_pct", "undershoot_pct", "settle5_ms",
                "ss_mean", "ss_std", "i_peak_A", "rise10_ms", "rise90_ms")
    out = dict(ms[0])  # copy non-numeric (file, side, target)
    for k in keys_avg:
        out[k] = sum(m[k] for m in ms) / len(ms)
    return out


def score_one(m: dict) -> float:
    """Per-capture penalty. Hard reject on safety; otherwise weighted sum."""
    if m is None:
        return 1e6
    if m["i_peak_A"] > I_PEAK_LIMIT:
        return 1e6
    if m["overshoot_pct"] > OS_LIMIT:
        return 1e6
    if abs(m["ss_mean"] - m["target"]) / max(abs(m["target"]), 1) > SS_ERR_LIMIT:
        return 1e6
    # Smooth penalties
    os_pen     = max(0.0, m["overshoot_pct"]  - 15.0) / 5.0
    settle_pen = max(0.0, m["settle5_ms"]     - 400.0) / 100.0
    sd_pen     = m["ss_std"] / 50.0
    us_pen     = max(0.0, m["undershoot_pct"] - 20.0) / 10.0
    return os_pen + settle_pen + sd_pen + us_pen


def evaluate(host: str, side: str, kp: float, ki: float, kd: float,
             out_dir: Path, tag: str) -> tuple[float, list[dict]]:
    """Apply gains, capture each step N_REPEATS times, return
    (cost, [averaged_metrics_per_step])."""
    if not check_vesc_ok(host):
        raise RuntimeError(
            "vesc_ok=0 (firmware watchdog tripped); power-cycle the ESP32.")
    if not apply_gains(host, side, kp, ki, kd):
        print(f"  [{tag}] apply_gains FAILED", file=sys.stderr)
        return 1e6, []
    time.sleep(0.4)
    avg_ms = []
    cost = 0.0
    for e1 in STEP_TARGETS:
        reps = []
        for rep in range(N_REPEATS):
            csv_path = out_dir / f"{tag}_{side}_{e1}_r{rep+1}.csv"
            if not capture_step(host, side, e1, csv_path):
                print(f"  [{tag}] capture {e1} rep{rep+1} FAILED",
                      file=sys.stderr)
                return 1e6, avg_ms
            m = analyze_metrics(str(csv_path), side=side)
            if m is None:
                return 1e6, avg_ms
            reps.append(m)
            time.sleep(RECOVER_S)
        m_avg = avg_metrics(reps)
        avg_ms.append(m_avg)
        cost += score_one(m_avg)
    return cost, avg_ms


def log_trial(log_csv: Path, trial: int, label: str,
              kp: float, ki: float, kd: float,
              cost: float, ms: list[dict]) -> None:
    new = not log_csv.exists()
    with open(log_csv, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow([
                "trial", "label", "kp", "ki", "kd", "cost",
                "OS_500", "settle_500", "ss_std_500", "US_500", "ipk_500",
                "OS_1500", "settle_1500", "ss_std_1500", "US_1500", "ipk_1500",
            ])
        row = [trial, label, f"{kp:.6g}", f"{ki:.6g}", f"{kd:.6g}", f"{cost:.3f}"]
        for m in ms:
            row += [f"{m['overshoot_pct']:.1f}",
                    f"{m['settle5_ms']:.0f}",
                    f"{m['ss_std']:.0f}",
                    f"{m['undershoot_pct']:.1f}",
                    f"{m['i_peak_A']:.2f}"]
        while len(row) < 16:
            row.append("")
        w.writerow(row)


def coord_descent(host: str, side: str, kp0: float, ki0: float, kd0: float,
                  out_dir: Path, cycles: int = 2) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    log_csv = out_dir / "log.csv"
    trial = 0

    # Warm-up: one discarded 1500 capture so the first scored trial isn't cold.
    print("[warm-up] discarded capture at 1500 ERPM", flush=True)
    if check_vesc_ok(host):
        warm_csv = out_dir / "warmup.csv"
        capture_step(host, side, 1500, warm_csv)
        time.sleep(RECOVER_S)

    def eval_and_log(kp, ki, kd, label):
        nonlocal trial
        trial += 1
        tag = f"t{trial:02d}"
        print(f"[trial {trial:2d}] {label:14s} kp={kp:.4g} ki={ki:.4g} kd={kd:.4g}",
              flush=True)
        cost, ms = evaluate(host, side, kp, ki, kd, out_dir, tag)
        log_trial(log_csv, trial, label, kp, ki, kd, cost, ms)
        if cost >= 1e6:
            print("    REJECT (hard limit)", flush=True)
        else:
            line = "    " + " ".join(
                f"{e1}: OS={m['overshoot_pct']:.0f}% st={m['settle5_ms']:.0f}ms "
                f"sd=±{m['ss_std']:.0f} US={m['undershoot_pct']:.0f}% "
                f"ipk={m['i_peak_A']:.1f}A"
                for e1, m in zip(STEP_TARGETS, ms)
            )
            print(line + f"  -> J={cost:.2f}", flush=True)
        return cost

    # Baseline
    best_kp, best_ki, best_kd = kp0, ki0, kd0
    best_cost = eval_and_log(best_kp, best_ki, best_kd, "baseline")

    factors = (0.7, 1.5)
    for cycle in range(1, cycles + 1):
        print(f"\n=== cycle {cycle} ===", flush=True)
        improved = False

        # kp
        for f in factors:
            kp = clamp(best_kp * f, KP_MIN, KP_MAX)
            if abs(kp - best_kp) < 1e-7:
                continue
            c = eval_and_log(kp, best_ki, best_kd, f"kp*{f}")
            if c < best_cost - 2.0:
                best_kp, best_cost, improved = kp, c, True

        # ki
        for f in factors:
            ki = clamp(best_ki * f, KI_MIN, KI_MAX)
            if abs(ki - best_ki) < 1e-7:
                continue
            c = eval_and_log(best_kp, ki, best_kd, f"ki*{f}")
            if c < best_cost - 2.0:
                best_ki, best_cost, improved = ki, c, True

        # kd (additive moves; multiplicative breaks at 0)
        kd_candidates = []
        if best_kd < 1e-6:
            kd_candidates = [0.0005]
        else:
            for f in factors:
                kd_candidates.append(clamp(best_kd * f, KD_MIN, KD_MAX))
        for kd in kd_candidates:
            if abs(kd - best_kd) < 1e-7:
                continue
            c = eval_and_log(best_kp, best_ki, kd, f"kd={kd:.4g}")
            if c < best_cost - 2.0:
                best_kd, best_cost, improved = kd, c, True

        if not improved:
            print(f"\nno improvement in cycle {cycle}; stopping.", flush=True)
            break

    # Final verify
    print(f"\n=== verify best kp={best_kp:.4g} ki={best_ki:.4g} kd={best_kd:.4g} ===",
          flush=True)
    verify_cost = eval_and_log(best_kp, best_ki, best_kd, "verify")

    best = {
        "kp": best_kp, "ki": best_ki, "kd": best_kd,
        "cost": best_cost, "verify_cost": verify_cost,
        "trials": trial,
    }
    (out_dir / "best.txt").write_text(
        f"kp={best['kp']:.6g}\nki={best['ki']:.6g}\nkd={best['kd']:.6g}\n"
        f"cost={best['cost']:.3f} verify_cost={best['verify_cost']:.3f}\n"
        f"trials={best['trials']}\n"
    )
    return best


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", required=True)
    p.add_argument("--side", choices=("L", "R"), required=True)
    p.add_argument("--kp", type=float, required=True)
    p.add_argument("--ki", type=float, required=True)
    p.add_argument("--kd", type=float, required=True)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--cycles", type=int, default=2)
    a = p.parse_args(argv)

    out = Path(a.out_dir)
    best = coord_descent(a.host, a.side, a.kp, a.ki, a.kd, out, cycles=a.cycles)
    print(f"\nDONE side={a.side} best kp={best['kp']:.4g} "
          f"ki={best['ki']:.4g} kd={best['kd']:.4g} J={best['cost']:.2f} "
          f"(verify J={best['verify_cost']:.2f})")
    print(f"results in {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
