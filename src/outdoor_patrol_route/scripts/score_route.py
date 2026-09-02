#!/usr/bin/env python3
# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
r"""Score recorded route files against the sim road's ground-truth centerline.

Covers validation runs **R1** (the teach pass produced a usable route) and
**R2** (the base_link correction is actually doing something).

R2 is the one with teeth. Record the same pass three ways and compare:

===================  ===================================================
``odometry_global``  base_link via navsat_transform's TF lever arm
``fix_lever_arm``    base_link via an explicit rotation of the TF offset
``raw_antenna``      the antenna, uncorrected -- the control
===================  ===================================================

The two corrected files should sit within a few centimetres of the true
centerline. The control should sit ~0.42 m to the RIGHT of it, because that is
where the antenna is (outdoor_patrol_bringup/config/chassis.yaml). **If all
three score the same, the correction is not wired** -- which is exactly the
failure this run exists to catch, and which a single-file check cannot see.

Geodetic samples are converted to the world frame through an exact WGS84
ECEF -> ENU transform about the route's own datum. The sim world declares that
same datum, so the result is directly comparable with Gazebo ground truth.

Usage::

    score_route.py --centerline patrol_road_centerline.yaml \\
        --route route_odometry_global.yaml \\
        --route route_fix_lever_arm.yaml \\
        --route route_raw_antenna.yaml
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
import yaml

# WGS84.
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

# Pass thresholds for a corrected route against the true centerline. RMS is
# the meaningful number -- it is what recording quality actually is. The peak
# is one worst sample out of ~130 and is driven by the 2 cm fix noise and by
# EKF transients at the corners, so it is set from measurement (0.083, 0.085,
# 0.093, 0.096, 0.126 m over five runs) rather than at a round number that
# fails one run in five. Both are an order of magnitude below the 0.42 m error
# the base_link correction removes, which is the thing under test.
DEFAULT_MAX_LATERAL_RMS_M = 0.05
DEFAULT_MAX_LATERAL_M = 0.15
DEFAULT_MIN_CONTROL_OFFSET_M = 0.30

CORRECTED_SOURCES = ('odometry_global', 'fix_lever_arm')
CONTROL_SOURCE = 'raw_antenna'


def geodetic_to_ecef(lat_deg, lon_deg, alt):
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    n = _A / np.sqrt(1.0 - _E2 * np.sin(lat) ** 2)
    x = (n + alt) * np.cos(lat) * np.cos(lon)
    y = (n + alt) * np.cos(lat) * np.sin(lon)
    z = (n * (1.0 - _E2) + alt) * np.sin(lat)
    return np.stack([x, y, z], axis=-1)


def geodetic_to_enu(lat_deg, lon_deg, alt, datum):
    """Exact local ENU metres about `datum` = (lat, lon, alt)."""
    d_lat, d_lon, d_alt = datum
    origin = geodetic_to_ecef(np.array(d_lat), np.array(d_lon), np.array(d_alt))
    point = geodetic_to_ecef(np.asarray(lat_deg), np.asarray(lon_deg),
                             np.asarray(alt))
    delta = point - origin

    lat = math.radians(d_lat)
    lon = math.radians(d_lon)
    east = np.array([-math.sin(lon), math.cos(lon), 0.0])
    north = np.array([-math.sin(lat) * math.cos(lon),
                      -math.sin(lat) * math.sin(lon), math.cos(lat)])
    return np.stack([delta @ east, delta @ north], axis=-1)


class Centerline:
    """Ground-truth road centerline, with a signed lateral axis."""

    def __init__(self, path: str):
        with open(path) as handle:
            road = yaml.safe_load(handle)['road']
        samples = road['samples']
        if road.get('loop'):
            samples = samples[:-1]
        self.xy = np.array([[s['x'], s['y']] for s in samples])
        self.yaw = np.array([s['yaw'] for s in samples])
        self.station = np.array([s['s'] for s in samples])
        self.length = float(road['length_m'])
        self.normal = np.stack([-np.sin(self.yaw), np.cos(self.yaw)], axis=1)
        self.road = road

    def lateral(self, points: np.ndarray):
        """Signed lateral offset of each point (+ left), and its station."""
        dx = points[:, 0][:, None] - self.xy[None, :, 0]
        dy = points[:, 1][:, None] - self.xy[None, :, 1]
        nearest = np.argmin(dx * dx + dy * dy, axis=1)
        rel = points - self.xy[nearest]
        lateral = np.einsum('ij,ij->i', rel, self.normal[nearest])
        return lateral, self.station[nearest]


def load_route(path: str):
    with open(path) as handle:
        route = yaml.safe_load(handle)
    if route.get('version') != 1:
        raise SystemExit('%s: unsupported schema version %r'
                         % (path, route.get('version')))
    samples = route['samples']
    datum = route['datum']
    xy = geodetic_to_enu(
        np.array([s['lat'] for s in samples]),
        np.array([s['lon'] for s in samples]),
        np.zeros(len(samples)),
        (datum['latitude'], datum['longitude'], 0.0))
    return route, samples, xy


def score(path: str, centerline: Centerline, max_lateral: float,
          max_lateral_rms: float):
    route, samples, xy = load_route(path)
    lateral, station = centerline.lateral(xy)

    steps = np.hypot(*(np.diff(xy, axis=0).T))
    closure = float(np.hypot(*(xy[-1] - xy[0])))
    classes = {}
    for s in samples:
        classes[s['fix']] = classes.get(s['fix'], 0) + 1

    result = {
        'file': os.path.basename(path),
        'source': route.get('source'),
        'frame': route.get('frame'),
        'loop': bool(route.get('loop')),
        'samples': len(samples),
        'span_m': float(np.sum(steps)),
        'spacing_min_m': float(np.min(steps)),
        'spacing_max_m': float(np.max(steps)),
        'closure_m': closure,
        'lateral_mean_m': float(np.mean(lateral)),
        'lateral_rms_m': float(np.sqrt(np.mean(lateral ** 2))),
        'lateral_max_abs_m': float(np.max(np.abs(lateral))),
        'fix_classes': classes,
        'stations_covered_m': float(np.ptp(station)),
    }
    result['is_control'] = route.get('source') == CONTROL_SOURCE
    if not result['is_control']:
        result['pass'] = (result['lateral_max_abs_m'] <= max_lateral
                          and result['lateral_rms_m'] <= max_lateral_rms)
    return result


def report(results, max_lateral: float, max_lateral_rms: float,
           min_control_offset: float):
    print('R1 -- teach pass produced a usable route')
    header = ('%-22s %-16s %5s %8s %9s %9s %9s'
              % ('file', 'source', 'n', 'span m', 'lat mean', 'lat rms',
                 'lat max'))
    print(header)
    print('-' * len(header))
    for r in results:
        print('%-22s %-16s %5d %8.2f %+9.3f %9.3f %9.3f'
              % (r['file'], r['source'], r['samples'], r['span_m'],
                 r['lateral_mean_m'], r['lateral_rms_m'],
                 r['lateral_max_abs_m']))

    failures = []
    for r in results:
        if r['samples'] < 50:
            failures.append('%s: only %d samples' % (r['file'], r['samples']))
        if r['spacing_max_m'] > 3.0:
            failures.append('%s: %.2f m gap between stations'
                            % (r['file'], r['spacing_max_m']))
        if r['loop'] and r['closure_m'] > 3.0:
            failures.append('%s: loop: true but ends %.2f m from the start'
                            % (r['file'], r['closure_m']))
        if not r['is_control'] and r['lateral_rms_m'] > max_lateral_rms:
            failures.append('%s: lateral RMS %.3f m exceeds %.3f m'
                            % (r['file'], r['lateral_rms_m'], max_lateral_rms))
        if not r['is_control'] and r['lateral_max_abs_m'] > max_lateral:
            failures.append('%s: peak lateral %.3f m exceeds %.3f m'
                            % (r['file'], r['lateral_max_abs_m'], max_lateral))

    print()
    print('R2 -- base_link correction is doing something')
    corrected = [r for r in results if not r['is_control']]
    control = [r for r in results if r['is_control']]
    if not corrected or not control:
        print('  SKIPPED: needs at least one corrected route and one '
              '%s control' % CONTROL_SOURCE)
    else:
        ctl = control[0]
        print('  control  %-16s mean lateral %+.3f m (antenna is right of '
              'base_link, so this should be negative)'
              % (ctl['source'], ctl['lateral_mean_m']))
        if abs(ctl['lateral_mean_m']) < min_control_offset:
            failures.append(
                'the %s control is only %.3f m off the centerline; it should '
                'show the full antenna offset, so the recording chain is not '
                'what this test assumes'
                % (ctl['source'], abs(ctl['lateral_mean_m'])))
        for r in corrected:
            gain = abs(ctl['lateral_mean_m']) - abs(r['lateral_mean_m'])
            print('  corrected %-16s mean lateral %+.3f m -> %.3f m better '
                  'than the control' % (r['source'], r['lateral_mean_m'], gain))
            if gain < min_control_offset:
                failures.append(
                    '%s scores no better than the uncorrected control '
                    '(%.3f m) -- the base_link correction is not wired'
                    % (r['source'], gain))

    print()
    if failures:
        print('FAIL')
        for line in failures:
            print('  - %s' % line)
        return 1
    print('PASS: %d route(s) within %.2f m RMS of the true centerline; the '
          'uncorrected control is %.2f m off, so the correction is '
          'demonstrated'
          % (len(corrected), max_lateral_rms,
             abs(control[0]['lateral_mean_m']) if control else 0.0))
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--centerline', required=True)
    parser.add_argument('--route', action='append', required=True)
    parser.add_argument('--max-lateral', type=float,
                        default=DEFAULT_MAX_LATERAL_M)
    parser.add_argument('--max-lateral-rms', type=float,
                        default=DEFAULT_MAX_LATERAL_RMS_M)
    parser.add_argument('--min-control-offset', type=float,
                        default=DEFAULT_MIN_CONTROL_OFFSET_M)
    parser.add_argument('--json', help='also write the raw numbers here')
    args = parser.parse_args(argv)

    centerline = Centerline(args.centerline)
    results = [score(path, centerline, args.max_lateral,
                     args.max_lateral_rms)
               for path in args.route]

    if args.json:
        with open(args.json, 'w') as handle:
            json.dump(results, handle, indent=2)

    return report(results, args.max_lateral, args.max_lateral_rms,
                  args.min_control_offset)


if __name__ == '__main__':
    sys.exit(main())
