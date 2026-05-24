#!/usr/bin/env python3
"""Summarise step-response CSVs captured by ``vesc_tune.py``.

For each CSV the script reports rise time (10 %, 90 %), overshoot,
undershoot, settle time inside ±5 %, steady-state mean / std and the
peak motor current. Side is auto-detected from whichever ``tgt_*``
column is the first non-zero one in the file (override with --side).

Examples
--------
    # Single file
    scripts/vesc_tune/analyze_step.py runs/step_L_0-3000_v4.csv

    # Whole sweep, sorted
    scripts/vesc_tune/analyze_step.py runs/step_*_v4.csv

    # CSV output for further processing
    scripts/vesc_tune/analyze_step.py --format csv runs/*.csv > metrics.csv
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import statistics as st
import sys
from typing import Optional


def load(path: str) -> list[dict]:
    with open(path) as f:
        return [{k: float(v) for k, v in row.items()}
                for row in csv.DictReader(f)]


def detect_side(rows: list[dict]) -> Optional[str]:
    for r in rows:
        if r["tgt_l"] != 0:
            return "L"
        if r["tgt_r"] != 0:
            return "R"
    return None


def metrics(path: str, side: Optional[str] = None) -> Optional[dict]:
    rows = load(path)
    if not rows:
        return None
    if side is None:
        side = detect_side(rows)
    if side is None:
        return None
    tgt_c = f"tgt_{side.lower()}"
    erpm_c = f"erpm_{side.lower()}"
    i_c = f"i_{side.lower()}"

    t_step = next((r["t_us"] for r in rows if r[tgt_c] != 0), None)
    if t_step is None:
        return None
    target = next(r[tgt_c] for r in rows if r[tgt_c] != 0)
    t_end = next((r["t_us"] for r in rows if r["t_us"] > t_step
                 and r[tgt_c] == 0), None)

    during = [r for r in rows
              if r["t_us"] >= t_step and (t_end is None or r["t_us"] < t_end)]
    if not during:
        return None

    sign = 1 if target > 0 else -1

    def rise_to(frac: float) -> Optional[float]:
        thr = frac * target
        for r in during:
            if sign * r[erpm_c] >= sign * thr:
                return (r["t_us"] - t_step) / 1e6
        return None

    rise10 = rise_to(0.1)
    rise90 = rise_to(0.9)

    n = len(during)
    ss = during[int(0.6 * n):]
    ss_mean = st.mean(r[erpm_c] for r in ss) if ss else float("nan")
    ss_std = st.stdev(r[erpm_c] for r in ss) if len(ss) > 1 else 0.0

    peak = max(during, key=lambda r: sign * r[erpm_c])[erpm_c]
    OS = (sign * (peak - target)) / abs(target) * 100

    # First trough after the overshoot
    trough = None
    pk_seen = False
    for r in during:
        if r[erpm_c] == peak:
            pk_seen = True
        elif pk_seen:
            trough = r[erpm_c] if trough is None else (
                min(trough, r[erpm_c]) if sign > 0 else max(trough, r[erpm_c]))
    US = (sign * (target - trough)) / abs(target) * 100 \
        if trough is not None else 0.0

    band = 0.05 * abs(target)
    last_out = t_step
    for r in during:
        if abs(r[erpm_c] - target) > band:
            last_out = r["t_us"]
    settle = (last_out - t_step) / 1e6

    i_peak = max(abs(r[i_c]) for r in during)

    return {
        "file": os.path.basename(path),
        "side": side,
        "target": int(target),
        "rise10_ms": (rise10 or 0) * 1000,
        "rise90_ms": (rise90 or 0) * 1000,
        "overshoot_pct": OS,
        "undershoot_pct": US,
        "settle5_ms": settle * 1000,
        "ss_mean": ss_mean,
        "ss_std": ss_std,
        "i_peak_A": i_peak,
    }


def print_table(results: list[dict]) -> None:
    if not results:
        print("(no results)", file=sys.stderr)
        return
    print(
        f"{'file':<32} {'side':>4} {'tgt':>5}  "
        f"{'rise10':>7} {'rise90':>7}  "
        f"{'OS%':>6} {'US%':>6}  {'settle':>7}  "
        f"{'ss_mean':>8} {'ss_std':>6}  {'i_pk':>5}"
    )
    for r in results:
        print(
            f"{r['file']:<32} {r['side']:>4} {r['target']:>5}  "
            f"{r['rise10_ms']:>6.0f}ms {r['rise90_ms']:>6.0f}ms  "
            f"{r['overshoot_pct']:>5.1f}% {r['undershoot_pct']:>5.1f}%  "
            f"{r['settle5_ms']:>6.0f}ms  "
            f"{r['ss_mean']:>7.0f} ±{r['ss_std']:>4.0f}  "
            f"{r['i_peak_A']:>4.2f}A"
        )


def print_csv(results: list[dict]) -> None:
    if not results:
        return
    w = csv.DictWriter(sys.stdout, fieldnames=list(results[0].keys()))
    w.writeheader()
    for r in results:
        w.writerow(r)


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("files", nargs="+",
                   help="CSV files (globs allowed)")
    p.add_argument("--side", choices=("L", "R"),
                   help="Force side instead of auto-detect")
    p.add_argument("--format", choices=("table", "csv"), default="table")
    args = p.parse_args(argv)

    paths: list[str] = []
    for pat in args.files:
        matched = sorted(glob.glob(pat)) or [pat]
        for m in matched:
            if os.path.isfile(m):
                paths.append(m)

    if not paths:
        print("no files matched", file=sys.stderr)
        return 1

    results = [m for m in (metrics(p, args.side) for p in paths) if m]

    if args.format == "csv":
        print_csv(results)
    else:
        print_table(results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
