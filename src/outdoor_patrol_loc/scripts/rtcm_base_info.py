#!/usr/bin/env python3
# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Report which base station is correcting us, and how far away it is.

Subscribes to ``/rtcm`` and decodes the station-description messages the
caster interleaves with the observations:

* **1005 / 1006** -- the base antenna reference point in ECEF. The only
  source of an actual distance.
* **1033** -- receiver and antenna descriptors, which is how a virtual base
  usually gives itself away.

## Why the distance may not mean what it looks like

If the mountpoint is a **VRS** (virtual reference station -- Point One's
``AUTO``, Trimble VRS, most network-RTK services), the caster synthesises a
base a few tens of metres from wherever your uploaded GGA says you are. The
baseline is then ~0 **by construction**, and reading it as "we are close to
the reference network, so corrections are excellent" is exactly wrong: the
real stations are tens of kilometres away and the interpolation between them
is what sets the error.

This script flags the giveaways rather than making you spot them:

* coordinates that are suspiciously round (a surveyed monument has arbitrary
  decimals; a synthesised point is snapped)
* a null antenna descriptor (``ADVNULLANTENNA``, ``NULLANTENNA``)
* a baseline under a kilometre, which no real deployment would bother with

For a *physical* base the distance does matter, at roughly 1 cm of added
error per 10 km of baseline.

Usage::

    ros2 run outdoor_patrol_loc rtcm_base_info.py      # 30 s, then reports
    rtcm_base_info.py --seconds 60 --topic /rtcm
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix

# WGS84.
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

#: Antenna descriptors that mean "there is no physical antenna here".
_NULL_ANTENNAS = ('ADVNULLANTENNA', 'NULLANTENNA', 'ADVNULL')

#: Below this, a "base station" is almost certainly synthesised beside us.
_VRS_BASELINE_M = 1000.0


def _bits(data: bytes, pos: int, length: int) -> int:
    """Unsigned big-endian bitfield, as RTCM3 packs them."""
    value = 0
    for k in range(length):
        bit = pos + k
        value = (value << 1) | ((data[bit >> 3] >> (7 - (bit & 7))) & 1)
    return value


def _signed_bits(data: bytes, pos: int, length: int) -> int:
    value = _bits(data, pos, length)
    return value - (1 << length) if value & (1 << (length - 1)) else value


def iter_rtcm(buffer: bytes):
    """Yield (message_type, payload) for each framed RTCM3 message.

    Frame: 0xD3, 6 reserved bits + 10-bit length, payload, 24-bit CRC. The
    CRC is not checked -- the receiver does that, and a corrupt frame here
    only costs one skipped report.
    """
    i = 0
    while i + 3 < len(buffer):
        if buffer[i] != 0xD3:
            i += 1
            continue
        length = ((buffer[i + 1] & 0x03) << 8) | buffer[i + 2]
        if i + 3 + length + 3 > len(buffer):
            break
        payload = buffer[i + 3:i + 3 + length]
        if len(payload) >= 2:
            yield ((payload[0] << 4) | (payload[1] >> 4), payload)
        i += 3 + length + 3


def ecef_to_lla(x: float, y: float, z: float):
    """ECEF metres -> (lat_deg, lon_deg, height_m), Bowring iteration."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _E2))
    height = 0.0
    for _ in range(12):
        n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
        height = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - _E2 * n / (n + height)))
    return math.degrees(lat), math.degrees(lon), height


def lla_to_ecef(lat_deg: float, lon_deg: float, height: float):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(lat) ** 2)
    return ((n + height) * math.cos(lat) * math.cos(lon),
            (n + height) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - _E2) + height) * math.sin(lat))


def looks_synthesised(lat: float, lon: float, height: float,
                      antenna: str, baseline_m):
    """Reasons to think this base is virtual rather than a real monument."""
    reasons = []
    # A surveyed mark has arbitrary decimals; a synthesised point is snapped.
    if all(abs(v * 1e4 - round(v * 1e4)) < 1e-6 for v in (lat, lon)):
        reasons.append('coordinates are round to 4+ decimal places')
    if abs(height - round(height)) < 1e-6:
        reasons.append('height is an exact whole number of metres')
    if antenna and any(k in antenna.upper() for k in _NULL_ANTENNAS):
        reasons.append('null antenna descriptor (%s)' % antenna.strip())
    if baseline_m is not None and baseline_m < _VRS_BASELINE_M:
        reasons.append('baseline is only %.0f m' % baseline_m)
    return reasons


class BaseInfo(Node):
    """Collect station descriptions until we have what we need."""

    def __init__(self, topic: str, fix_topic: str):
        super().__init__('rtcm_base_info')
        from rtcm_msgs.msg import Message as Rtcm

        self.base_ecef = None
        self.station_id = None
        self.antenna = ''
        self.receiver = ''
        self.types = {}
        self.frames = 0
        self.rover = None

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Rtcm, topic, self._on_rtcm, qos)
        self.create_subscription(NavSatFix, fix_topic, self._on_fix, qos)

    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:
            self.rover = (msg.latitude, msg.longitude, msg.altitude)

    def _on_rtcm(self, msg) -> None:
        for kind, payload in iter_rtcm(bytes(msg.message)):
            self.frames += 1
            self.types[kind] = self.types.get(kind, 0) + 1

            if kind in (1005, 1006) and len(payload) >= 19:
                self.station_id = _bits(payload, 12, 12)
                self.base_ecef = (_signed_bits(payload, 34, 38) * 1e-4,
                                  _signed_bits(payload, 74, 38) * 1e-4,
                                  _signed_bits(payload, 114, 38) * 1e-4)
            elif kind == 1033 and len(payload) >= 6:
                # station id (12) then a counted ASCII antenna descriptor.
                n = payload[3]
                if len(payload) >= 4 + n:
                    self.antenna = payload[4:4 + n].decode('ascii', 'replace')

    def ready(self) -> bool:
        return self.base_ecef is not None and self.rover is not None


def report(node: BaseInfo) -> int:
    if node.base_ecef is None:
        print('No RTCM 1005/1006 seen. Either corrections are not flowing, '
              'or this caster does not send station descriptions -- check '
              '`ros2 topic hz /rtcm` first.')
        return 1

    x, y, z = node.base_ecef
    lat, lon, height = ecef_to_lla(x, y, z)

    baseline = None
    if node.rover is not None:
        rx, ry, rz = lla_to_ecef(*node.rover)
        baseline = math.dist((x, y, z), (rx, ry, rz))

    print('Base station')
    print('  station id  : %s' % node.station_id)
    print('  ECEF        : %.4f %.4f %.4f' % (x, y, z))
    print('  LLA         : %.7f, %.7f, %.2f m' % (lat, lon, height))
    if node.antenna:
        print('  antenna     : %s' % node.antenna.strip())
    print('  RTCM types  : %s'
          % ', '.join(str(k) for k in sorted(node.types)))

    if node.rover is None:
        print()
        print('  No rover fix, so no baseline. Is the GNSS driver running?')
        return 0

    print()
    print('Rover')
    print('  LLA         : %.7f, %.7f, %.1f m' % node.rover)
    print()
    print('Baseline      : %.3f km' % (baseline / 1000.0))

    reasons = looks_synthesised(lat, lon, height, node.antenna, baseline)
    print()
    if reasons:
        print('This base is VIRTUAL (VRS). Evidence:')
        for r in reasons:
            print('  - %s' % r)
        print()
        print('So the baseline above is an artefact of how the VRS is')
        print('generated, NOT a measure of correction quality. The real')
        print('reference stations are tens of km away and the network')
        print('interpolation between them is what sets your error.')
        print()
        print('  * send_gga must stay true -- the caster needs your position')
        print('    to synthesise the base at all.')
        print('  * The base position jumps as you travel and it re-synthesises.')
        print('  * A VRS usually beats a distant physical base, because the')
        print('    network models the atmosphere rather than extrapolating')
        print('    from one station. Confirm with a soak, do not assume.')
    else:
        added = 1.0 + (baseline / 1000.0) / 10.0
        print('This looks like a PHYSICAL base, so the distance matters:')
        print('  roughly 1 cm + 1 cm per 10 km of baseline')
        print('  -> ~%.1f cm of baseline-dependent error at %.1f km'
              % (added, baseline / 1000.0))
        if baseline > 30000.0:
            print('  That is a long baseline. Expect the poor end of any')
            print('  quoted accuracy range, and RTK float rather than fixed.')
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--topic', default='/rtcm')
    parser.add_argument('--fix-topic', default='/um982_driver/fix')
    parser.add_argument('--seconds', type=float, default=30.0,
                        help='How long to listen. 1005 is usually sent every '
                             '10 s or so, so allow at least 30.')
    args = parser.parse_args(argv)

    rclpy.init()
    node = BaseInfo(args.topic, args.fix_topic)
    deadline = time.time() + args.seconds
    try:
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            # 1005 arrives infrequently; stop as soon as it does.
            if node.ready() and node.antenna:
                break
        return report(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
