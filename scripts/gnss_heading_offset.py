#!/usr/bin/env python3
"""Measure the GNSS dual-antenna mounting angle -> ``yaw_offset``.

Phase 2 of the field validation compares ``/gnss/heading`` against the course
the robot actually drove, taken from ``/odometry/global``. That comparison is
useful as a gate but poor as a *measurement*: it latches a single sample after
a short chord, and the course it uses comes out of ``ekf_global``, which is
itself being fed the heading under test.

This script measures the same quantity without either of those weaknesses. It
reads only the receiver's own NMEA:

* ``$GNGGA``  -> RTK position, from which the course actually driven is derived
* ``$KSXT``   -> the ANT1->ANT2 baseline heading and its solution quality

No EKF, no wheel odometry, no IMU, and no ``yaw_offset`` -- so the result is
independent of the thing being calibrated.

The maths, all in compass degrees (CW from true north):

    H = KSXT heading          (direction of the ANT1->ANT2 baseline)
    C = course over ground    (direction the robot actually travelled)

``um982_driver`` publishes the baseline as an ENU yaw (``yaw = 90 - H``) and
``heading_to_imu`` then adds ``yaw_offset``. For the output to equal the ENU
yaw of the robot's nose (``90 - C``):

    yaw_offset + 90 - H = 90 - C   =>   yaw_offset = H - C

So the offset *is* the angle between the antenna baseline and the direction of
travel. Drive straight forward and this script reports it directly.

Usage (inside the deploy container, while driving straight forward ~8 m):

    ros2 run ... # not a package entry point; run it directly:
    python3 gnss_heading_offset.py --seconds 60

Then put the printed value into ``yaw_offset`` in
``src/outdoor_patrol_loc/config/heading_to_imu.yaml``.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from nmea_msgs.msg import Sentence
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

EARTH_R = 6371000.0

# Minimum straight-line distance for the course to mean anything. At 5 m a
# 0.25 m wobble is still only ~3 deg of course error; much below that and the
# measurement is no better than the gate it is replacing.
MIN_USEFUL_DIST_M = 5.0
# Speed above which a sample counts as "moving". Derived from position, so it
# has to clear RTK noise (~1.3 cm) over the smoothing window.
MOVING_SPEED_MS = 0.15
SPEED_WINDOW_S = 1.0


@dataclass
class Fix:
    t: float
    lat: float
    lon: float
    quality: int


@dataclass
class Heading:
    t: float
    deg: float
    quality: int
    ant2_sats: int
    ant1_sats: int


def nmea_to_deg(raw: str, hemi: str) -> Optional[float]:
    """Convert NMEA ddmm.mmmm / dddmm.mmmm + hemisphere to signed degrees."""
    if not raw or '.' not in raw:
        return None
    dot = raw.index('.')
    # Degrees are everything before the last two digits of the integer part.
    deg_digits = dot - 2
    if deg_digits <= 0:
        return None
    try:
        deg = float(raw[:deg_digits])
        minutes = float(raw[deg_digits:])
    except ValueError:
        return None
    val = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        val = -val
    return val


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def circular_mean(degs: List[float]) -> Tuple[float, float]:
    """Return (mean_deg, circular_std_deg) for a list of angles."""
    if not degs:
        return float('nan'), float('nan')
    s = sum(math.sin(math.radians(d)) for d in degs) / len(degs)
    c = sum(math.cos(math.radians(d)) for d in degs) / len(degs)
    mean = math.degrees(math.atan2(s, c)) % 360.0
    r = min(math.hypot(s, c), 1.0)
    # Mardia's circular standard deviation.
    std = math.degrees(math.sqrt(-2.0 * math.log(r))) if r > 1e-12 else float('inf')
    return mean, std


class Collector(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('gnss_heading_offset')
        self.fixes: List[Fix] = []
        self.headings: List[Heading] = []
        # The driver publishes NMEA best-effort; a reliable subscriber would
        # silently never match it.
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Sentence, topic, self._cb, qos)
        self._warned = False

    def _cb(self, msg: Sentence) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        text = msg.sentence
        g = re.search(r'GGA[^*]*', text)
        if g:
            f = g.group(0).split(',')
            if len(f) > 9:
                lat = nmea_to_deg(f[2], f[3])
                lon = nmea_to_deg(f[4], f[5])
                try:
                    q = int(f[6])
                except ValueError:
                    q = 0
                if lat is not None and lon is not None:
                    self.fixes.append(Fix(t, lat, lon, q))
            return
        k = re.search(r'KSXT[^*]*', text)
        if k:
            f = k.group(0).split(',')
            # f[5]=heading  f[11]=heading quality  f[12]=ANT2 sats  f[13]=ANT1
            if len(f) > 13:
                try:
                    hdg = float(f[5])
                    hq = int(f[11])
                    a2 = int(f[12])
                    a1 = int(f[13])
                except ValueError:
                    return
                self.headings.append(Heading(t, hdg, hq, a2, a1))


def analyse(fixes: List[Fix], headings: List[Heading], min_hq: int) -> int:
    if len(fixes) < 10:
        print(f'\nNot enough GGA fixes ({len(fixes)}). Is the stack running?')
        return 1

    lat0 = fixes[0].lat
    lon0 = fixes[0].lon
    cos_lat = math.cos(math.radians(lat0))

    def enu(f: Fix) -> Tuple[float, float]:
        e = math.radians(f.lon - lon0) * EARTH_R * cos_lat
        n = math.radians(f.lat - lat0) * EARTH_R
        return e, n

    pts = [(f.t, *enu(f), f.quality) for f in fixes]

    # Speed from a centred window, so RTK noise does not dominate.
    moving: List[Tuple[float, float, float, int]] = []
    for i, (t, e, n, q) in enumerate(pts):
        lo, hi = i, i
        while lo > 0 and t - pts[lo][0] < SPEED_WINDOW_S / 2:
            lo -= 1
        while hi < len(pts) - 1 and pts[hi][0] - t < SPEED_WINDOW_S / 2:
            hi += 1
        dt = pts[hi][0] - pts[lo][0]
        if dt <= 0:
            continue
        d = math.hypot(pts[hi][1] - pts[lo][1], pts[hi][2] - pts[lo][2])
        if d / dt >= MOVING_SPEED_MS:
            moving.append((t, e, n, q))

    if len(moving) < 10:
        print(f'\nOnly {len(moving)} moving samples. Did the robot drive? '
              f'(threshold {MOVING_SPEED_MS} m/s)')
        return 1

    t0, e0, n0, _ = moving[0]
    t1, e1, n1, _ = moving[-1]
    de, dn = e1 - e0, n1 - n0
    dist = math.hypot(de, dn)

    # Course over ground, compass degrees CW from north.
    course = math.degrees(math.atan2(de, dn)) % 360.0

    # Straightness: perpendicular deviation from the start->end chord.
    devs = []
    if dist > 0.1:
        ux, uy = de / dist, dn / dist
        for _, e, n, _q in moving:
            devs.append(abs((e - e0) * uy - (n - n0) * ux))
    rms_dev = math.sqrt(sum(d * d for d in devs) / len(devs)) if devs else 0.0
    max_dev = max(devs) if devs else 0.0

    # Headings inside the moving window that met the quality bar.
    usable = [h for h in headings if t0 <= h.t <= t1 and h.quality >= min_hq]
    rejected = [h for h in headings if t0 <= h.t <= t1 and h.quality < min_hq]
    if not usable:
        print(f'\nNo KSXT headings with quality >= {min_hq} while moving '
              f'({len(rejected)} rejected). Check ANT2.')
        return 1

    hdg_mean, hdg_std = circular_mean([h.deg for h in usable])
    offset = wrap180(hdg_mean - course)
    nearest = round(offset / 90.0) * 90.0
    residual = wrap180(offset - nearest)

    fix_q = {}
    for _, _, _, q in moving:
        fix_q[q] = fix_q.get(q, 0) + 1
    hq_counts = {}
    for h in usable + rejected:
        hq_counts[h.quality] = hq_counts.get(h.quality, 0) + 1

    print('\n' + '=' * 62)
    print('  GNSS heading offset measurement')
    print('=' * 62)
    print(f'  moving samples      {len(moving)}  over {t1 - t0:.1f} s')
    print(f'  distance driven     {dist:.2f} m')
    print(f'  straightness        RMS {rms_dev:.3f} m   max {max_dev:.3f} m')
    print(f'  GGA quality         {fix_q}   (4 = RTK fixed)')
    print(f'  KSXT hdg quality    {hq_counts}   (3 = RTK fixed heading)')
    print(f'  ANT2 / ANT1 sats    {usable[-1].ant2_sats} / {usable[-1].ant1_sats}')
    print('-' * 62)
    print(f'  baseline heading H  {hdg_mean:8.2f} deg  (compass, +/- {hdg_std:.2f})')
    print(f'  course driven    C  {course:8.2f} deg  (compass)')
    print('-' * 62)
    print(f'  yaw_offset = H - C  {offset:8.2f} deg')
    print(f'  nearest clean mount {nearest:8.1f} deg   residual {residual:+.2f} deg')
    print('=' * 62)

    ok = True
    if dist < MIN_USEFUL_DIST_M:
        print(f'  WARNING: only {dist:.2f} m driven; want >= {MIN_USEFUL_DIST_M:.0f} m.')
        ok = False
    if rms_dev > 0.25:
        print(f'  WARNING: path bowed by {rms_dev:.2f} m RMS -- course may be off.')
        ok = False
    if hdg_std > 5.0:
        print(f'  WARNING: heading scattered by {hdg_std:.1f} deg.')
        ok = False
    if abs(residual) > 15.0:
        print(f'  WARNING: {residual:+.1f} deg from a clean 90 deg mount. Either the')
        print('           antennas are mounted at an odd angle, or the run was poor.')
        ok = False

    print()
    if ok:
        print('  Looks clean. Set in heading_to_imu.yaml:')
        print(f'      yaw_offset: {math.radians(nearest):.10f}   # {nearest:.0f} deg')
        print(f'  (raw measurement {math.radians(offset):.10f} = {offset:.2f} deg)')
    else:
        print('  Re-run before trusting this. Raw measurement was:')
        print(f'      yaw_offset: {math.radians(offset):.10f}   # {offset:.2f} deg')
    print()
    return 0 if ok else 2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seconds', type=float, default=60.0,
                    help='capture duration (default 60)')
    ap.add_argument('--topic', default='/um982_driver/nmea_sentence')
    ap.add_argument('--min-heading-quality', type=int, default=2,
                    help='2 = RTK float heading, 3 = RTK fixed (default 2)')
    args = ap.parse_args()

    rclpy.init()
    node = Collector(args.topic)

    print(f'Capturing {args.seconds:.0f} s from {args.topic}')
    print('DRIVE STRAIGHT FORWARD NOW (~8 m, steady, no turns).\n')

    start = node.get_clock().now().nanoseconds / 1e9
    next_tick = 0.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = node.get_clock().now().nanoseconds / 1e9
            elapsed = now - start
            if elapsed >= args.seconds:
                break
            if elapsed >= next_tick:
                next_tick += 5.0
                d = 0.0
                if len(node.fixes) > 1:
                    a, b = node.fixes[0], node.fixes[-1]
                    cos_lat = math.cos(math.radians(a.lat))
                    d = math.hypot(
                        math.radians(b.lon - a.lon) * EARTH_R * cos_lat,
                        math.radians(b.lat - a.lat) * EARTH_R)
                print(f'  {elapsed:5.1f}s  fixes={len(node.fixes):4d}  '
                      f'headings={len(node.headings):4d}  '
                      f'net displacement={d:5.2f} m')
    except KeyboardInterrupt:
        print('\ninterrupted -- analysing what was captured')

    rc = analyse(node.fixes, node.headings, args.min_heading_quality)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
