#!/usr/bin/env python3
"""Is the map frame rotated relative to true ENU?

``navsat_transform`` fixes the UTM->map rotation once, from the IMU heading
available at the moment it initialises, and (with no ``datum`` service exposed)
never revisits it. If it initialised while ``/gnss/heading`` was silent -- which
is what happens when ANT2 has no signal -- every position it emits afterwards is
rotated by the error in that initial heading.

That failure is invisible in any single topic. ``/gnss/heading`` looks right,
the EKF yaw agrees with it, and the GNSS fix is RTK-fixed centimetre-accurate.
Only when the *position track* is compared against the *raw GNSS track* does the
rotation appear -- and on screen it looks like the robot crabbing sideways.

So this captures both during one straight drive:

    course_map = direction travelled per /odometry/global   (map frame)
    course_enu = direction travelled per raw $GNGGA         (true ENU)
    rotation   = course_map - course_enu

``rotation`` near 0 means the map frame is sound and the crab has another
cause. A large, consistent ``rotation`` means the map frame is turned by that
much and the localisation stack has to be restarted while the heading is
healthy so that ``navsat_transform`` initialises correctly.

Run it while driving straight forward ~5 m:

    python3 /data/map_frame_check.py --seconds 45
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

EARTH_R = 6371000.0
MIN_DIST_M = 3.0


def nmea_to_deg(raw: str, hemi: str) -> Optional[float]:
    if not raw or '.' not in raw:
        return None
    dot = raw.index('.')
    dd = dot - 2
    if dd <= 0:
        return None
    try:
        val = float(raw[:dd]) + float(raw[dd:]) / 60.0
    except ValueError:
        return None
    return -val if hemi in ('S', 'W') else val


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def straightness(pts: List[Tuple[float, float]]) -> float:
    if len(pts) < 3:
        return 0.0
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-6:
        return 0.0
    ux, uy = (x1 - x0) / d, (y1 - y0) / d
    devs = [abs((x - x0) * uy - (y - y0) * ux) for x, y in pts]
    return math.sqrt(sum(v * v for v in devs) / len(devs))


class Both(Node):
    def __init__(self, odom_topic: str, nmea_topic: str) -> None:
        super().__init__('map_frame_check')
        self.odom: List[Tuple[float, float, float, float]] = []
        self.gnss: List[Tuple[float, float, float, int]] = []
        self.create_subscription(
            Odometry, odom_topic,
            self._odom_cb,
            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            Sentence, nmea_topic,
            self._nmea_cb,
            QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))

    def _odom_cb(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom.append(
            (t, p.x, p.y, math.degrees(yaw_from_quat(q.x, q.y, q.z, q.w))))

    def _nmea_cb(self, msg: Sentence) -> None:
        g = re.search(r'GGA[^*]*', msg.sentence)
        if not g:
            return
        f = g.group(0).split(',')
        if len(f) < 10:
            return
        lat = nmea_to_deg(f[2], f[3])
        lon = nmea_to_deg(f[4], f[5])
        if lat is None or lon is None:
            return
        try:
            q = int(f[6])
        except ValueError:
            q = 0
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gnss.append((t, lat, lon, q))


def analyse(odom, gnss) -> int:
    if len(odom) < 20 or len(gnss) < 10:
        print(f'\nNot enough data (odom={len(odom)}, gnss={len(gnss)}).')
        return 1

    # Overlapping window only -- comparing different stretches of the drive
    # would inject a difference that has nothing to do with frames.
    t0 = max(odom[0][0], gnss[0][0])
    t1 = min(odom[-1][0], gnss[-1][0])
    od = [o for o in odom if t0 <= o[0] <= t1]
    gn = [g for g in gnss if t0 <= g[0] <= t1]
    if len(od) < 20 or len(gn) < 10:
        print('\nTopics do not overlap in time; cannot compare.')
        return 1

    o_pts = [(o[1], o[2]) for o in od]
    o_d = math.hypot(o_pts[-1][0] - o_pts[0][0], o_pts[-1][1] - o_pts[0][1])
    # /odometry/global is ENU-convention: x=east, y=north.
    course_map = math.degrees(math.atan2(o_pts[-1][1] - o_pts[0][1],
                                         o_pts[-1][0] - o_pts[0][0]))

    lat0, lon0 = gn[0][1], gn[0][2]
    cl = math.cos(math.radians(lat0))
    g_pts = [(math.radians(g[2] - lon0) * EARTH_R * cl,
              math.radians(g[1] - lat0) * EARTH_R) for g in gn]
    g_d = math.hypot(g_pts[-1][0] - g_pts[0][0], g_pts[-1][1] - g_pts[0][1])
    course_enu = math.degrees(math.atan2(g_pts[-1][1] - g_pts[0][1],
                                         g_pts[-1][0] - g_pts[0][0]))

    rotation = wrap180(course_map - course_enu)
    mean_yaw = math.degrees(math.atan2(
        sum(math.sin(math.radians(o[3])) for o in od) / len(od),
        sum(math.cos(math.radians(o[3])) for o in od) / len(od)))

    qual = {}
    for g in gn:
        qual[g[3]] = qual.get(g[3], 0) + 1

    print('\n' + '=' * 64)
    print('  Map frame check')
    print('=' * 64)
    print(f'  window              {t1 - t0:.1f} s')
    print(f'  GGA quality         {qual}   (4 = RTK fixed)')
    print('-' * 64)
    print(f'  raw GNSS   distance {g_d:6.2f} m   straightness {straightness(g_pts):.3f} m RMS')
    print(f'             course   {course_enu:+7.1f} deg  (true ENU)')
    print(f'  /odometry/global    {o_d:6.2f} m   straightness {straightness(o_pts):.3f} m RMS')
    print(f'             course   {course_map:+7.1f} deg  (map frame)')
    print(f'             mean yaw {mean_yaw:+7.1f} deg')
    print('-' * 64)
    print(f'  MAP ROTATION        {rotation:+7.1f} deg')
    print(f'  crab (yaw - course) {wrap180(mean_yaw - course_map):+7.1f} deg')
    print(f'  scale ratio         {(o_d / g_d if g_d > 0 else 0):6.3f}   (want ~1.0)')
    print('=' * 64)

    if g_d < MIN_DIST_M:
        print(f'  Only {g_d:.2f} m driven; want >= {MIN_DIST_M:.0f} m. Re-run.')
        print()
        return 1

    if abs(rotation) <= 5.0:
        print('  The map frame is sound -- both tracks agree on the direction')
        print('  travelled. The crab has another cause; suspect the yaw fused')
        print('  into ekf_global rather than navsat_transform.')
    else:
        print(f'  The map frame is rotated by {rotation:+.1f} deg. navsat_transform')
        print('  fixes this rotation once, at start-up, from the heading then')
        print('  available -- and it started while ANT2 was dead and')
        print('  /gnss/heading was silent. It cannot recover on its own: there')
        print('  is no datum service to re-seed it.')
        print()
        print('  FIX -- restart the stack now that the heading is healthy:')
        print('      ssh robot "docker restart outdoor-patrol"')
        print('  Wait ~60 s, confirm heading is good, then re-run this check.')
    print()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seconds', type=float, default=45.0)
    ap.add_argument('--odom-topic', default='/odometry/global')
    ap.add_argument('--nmea-topic', default='/um982_driver/nmea_sentence')
    args = ap.parse_args()

    rclpy.init()
    node = Both(args.odom_topic, args.nmea_topic)
    print(f'Capturing {args.seconds:.0f} s')
    print('DRIVE STRAIGHT FORWARD NOW (~5 m is enough).\n')

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
                if len(node.odom) > 1:
                    a, b = node.odom[0], node.odom[-1]
                    d = math.hypot(b[1] - a[1], b[2] - a[2])
                print(f'  {el:5.1f}s  odom={len(node.odom):4d}  '
                      f'gnss={len(node.gnss):4d}  moved={d:5.2f} m')
    except KeyboardInterrupt:
        print('\ninterrupted -- analysing what was captured')

    rc = analyse(node.odom, node.gnss)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
