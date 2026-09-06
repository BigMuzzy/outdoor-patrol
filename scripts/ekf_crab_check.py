#!/usr/bin/env python3
"""Measure the on-screen "crab": EKF yaw vs the course the EKF itself travels.

RViz's Odometry display draws one arrow per ``Position Tolerance`` metres along
``/odometry/global``. Each arrow is placed at that message's position and
pointed along that message's yaw. So an arrow trail that walks diagonally
across the screen -- robot appearing to crab sideways -- means those two parts
of the *same* topic disagree: the yaw does not match the direction the
positions are moving.

This measures that disagreement directly, using only ``/odometry/global``, so
it reports precisely what RViz is drawing. Compare with
``gnss_heading_offset.py``, which answers a different question (is the antenna
mounting angle right?) from raw NMEA.

Run it while driving straight forward:

    python3 /data/ekf_crab_check.py --seconds 45

A healthy result is ``mean |crab|`` of a few degrees. A large steady crab means
the yaw feeding the map frame is wrong; a crab that *shrinks* over the run means
you are watching an estimator that has since converged, and the old arrows on
screen are stale.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

MOVING_SPEED_MS = 0.15
# Chord over which the course is computed. Long enough that RTK noise does not
# swamp it, short enough to still resolve a change during the run.
CHORD_M = 1.0


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class Crab(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('ekf_crab_check')
        self.samples: List[Tuple[float, float, float, float]] = []
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, topic, self._cb, qos)

    def _cb(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.samples.append(
            (t, p.x, p.y, math.degrees(yaw_from_quat(q.x, q.y, q.z, q.w))))


def analyse(s: List[Tuple[float, float, float, float]]) -> int:
    if len(s) < 20:
        print(f'\nOnly {len(s)} messages. Is /odometry/global publishing?')
        return 1

    rows: List[Tuple[float, float, float, float]] = []
    j = 0
    for i in range(len(s)):
        # Walk j forward until the chord from i is long enough.
        while j < len(s) - 1 and math.hypot(s[j][1] - s[i][1],
                                            s[j][2] - s[i][2]) < CHORD_M:
            j += 1
        d = math.hypot(s[j][1] - s[i][1], s[j][2] - s[i][2])
        dt = s[j][0] - s[i][0]
        if d < CHORD_M or dt <= 0 or d / dt < MOVING_SPEED_MS:
            continue
        course = math.degrees(math.atan2(s[j][2] - s[i][2], s[j][1] - s[i][1]))
        yaw = s[i][3]
        rows.append((s[i][0], yaw, course, wrap180(yaw - course)))

    if not rows:
        print(f'\nNo moving samples with a {CHORD_M:.1f} m chord. Did it drive?')
        return 1

    crabs = [r[3] for r in rows]
    t0 = rows[0][0]
    n = len(rows)
    first = crabs[:max(1, n // 3)]
    last = crabs[-max(1, n // 3):]

    def avg(a: List[float]) -> float:
        return sum(a) / len(a)

    print('\n' + '=' * 62)
    print('  EKF crab check  (what the RViz arrows are doing)')
    print('=' * 62)
    print(f'  samples             {n} over {rows[-1][0] - t0:.1f} s')
    print(f'  mean crab           {avg(crabs):+7.2f} deg')
    print(f'  mean |crab|         {avg([abs(c) for c in crabs]):7.2f} deg')
    print(f'  range               {min(crabs):+.1f} .. {max(crabs):+.1f} deg')
    print(f'  first third         {avg(first):+7.2f} deg')
    print(f'  last third          {avg(last):+7.2f} deg')
    print('-' * 62)
    print('  time    yaw     course    crab')
    step = max(1, n // 12)
    for r in rows[::step]:
        print(f'  {r[0] - t0:5.1f}  {r[1]:7.1f}  {r[2]:7.1f}  {r[3]:+7.1f}')
    print('=' * 62)

    m = avg([abs(c) for c in crabs])
    improving = abs(avg(last)) < abs(avg(first)) - 5.0
    if m <= 5.0:
        print('  VERDICT: healthy. Arrows point along the path; any crooked')
        print('           arrows still on screen are stale -- clear the trail.')
    elif improving:
        print(f'  VERDICT: converging ({avg(first):+.1f} -> {avg(last):+.1f} deg).')
        print('           The estimator is settling. Clear the trail, drive again,')
        print('           and re-check before changing any configuration.')
    else:
        print(f'  VERDICT: persistent {avg(crabs):+.1f} deg crab. The yaw feeding')
        print('           the map frame disagrees with the path being travelled.')
    print()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seconds', type=float, default=45.0)
    ap.add_argument('--topic', default='/odometry/global')
    args = ap.parse_args()

    rclpy.init()
    node = Crab(args.topic)
    print(f'Capturing {args.seconds:.0f} s from {args.topic}')
    print('DRIVE STRAIGHT FORWARD NOW.\n')

    start = node.get_clock().now().nanoseconds / 1e9
    tick = 0.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            el = node.get_clock().now().nanoseconds / 1e9 - start
            if el >= args.seconds:
                break
            if el >= tick:
                tick += 5.0
                d = 0.0
                if len(node.samples) > 1:
                    a, b = node.samples[0], node.samples[-1]
                    d = math.hypot(b[1] - a[1], b[2] - a[2])
                print(f'  {el:5.1f}s  msgs={len(node.samples):4d}  '
                      f'net displacement={d:5.2f} m')
    except KeyboardInterrupt:
        print('\ninterrupted -- analysing what was captured')

    rc = analyse(node.samples)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
