# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Read and write the teach-and-repeat route file.

The route is stored **geodetic** (lat/lon/alt), not in `map` XY, and every
sample is the pose of **base_link** -- not of the GNSS antenna. Both choices
are load-bearing:

*Geodetic, because `map` is not stable.* `navsat_transform` auto-sets its datum
on the first valid fix unless pinned, so a `map`-frame track silently shifts
between sessions. Storing lat/lon and projecting at load time makes the file
independent of whatever datum happens to be in force -- and the datum that WAS
in force is recorded alongside, so a mismatch is visible rather than silent.

*base_link, because the antenna is not the robot.* On this vehicle `gnss_link`
sits 0.28 m forward and 0.42 m right of `base_link`
(outdoor_patrol_bringup/config/chassis.yaml). Recording the raw fix would put
the "centerline" 0.42 m right of where the robot centre actually travelled,
and through a corner the antenna sweeps an arc while `base_link` barely
translates -- an error no constant offset can describe. Which correction path
produced a file is recorded in `source`, so an uncorrected file cannot be
mistaken for a corrected one.

Schema::

    version: 1
    recorded: 2026-09-02T01:23:45+00:00
    source: odometry_global          # or fix_lever_arm
    frame: base_link
    loop: true
    lane_half_width_m: 2.0
    datum: {latitude: ..., longitude: ..., altitude: ...}
    samples:
      - {lat, lon, alt, yaw, fix, sigma_h, shoulder_left_m, shoulder_right_m}

`yaw` is REP-103 (ENU, radians, CCW from east). `fix` is the coarse quality
class the sample was taken at; `sigma_h` the horizontal standard deviation the
driver reported, in metres. Per-sample shoulder widths are written even when
they are a constant, so multi-pass shoulder measurement can populate them later
without a format change.
"""

from __future__ import annotations

import datetime
import math
from dataclasses import dataclass, field

import yaml

SCHEMA_VERSION = 1

#: Recording paths, written into `source`.
SOURCE_ODOMETRY = 'odometry_global'   # /odometry/global, base_link by way of
#                                       navsat_transform's TF lever arm
SOURCE_FIX_LEVER_ARM = 'fix_lever_arm'  # raw fix minus R(yaw) * t_antenna
SOURCE_RAW_ANTENNA = 'raw_antenna'    # NOT base_link -- diagnostics only

#: Coarse fix-quality classes. Ordered worst to best; the follower and the
#: recorder both compare by index.
FIX_CLASSES = ('none', 'single', 'float', 'fixed')


@dataclass
class Sample:
    """One recorded station along the route."""

    lat: float
    lon: float
    alt: float
    yaw: float
    fix: str = 'fixed'
    sigma_h: float = 0.0
    shoulder_left_m: float = 1.0
    shoulder_right_m: float = 1.0


@dataclass
class Route:
    """A recorded route: a datum, a corridor width, and a list of samples."""

    datum: tuple = (0.0, 0.0, 0.0)
    loop: bool = False
    lane_half_width_m: float = 2.0
    source: str = SOURCE_ODOMETRY
    frame: str = 'base_link'
    recorded: str = ''
    samples: list = field(default_factory=list)

    @property
    def is_base_link(self) -> bool:
        """False for a diagnostics-only file recorded at the antenna."""
        return self.source != SOURCE_RAW_ANTENNA

    def worst_fix(self) -> str:
        """Lowest fix class present, or 'none' for an empty route."""
        if not self.samples:
            return 'none'
        return min((s.fix for s in self.samples),
                   key=lambda c: FIX_CLASSES.index(c)
                   if c in FIX_CLASSES else 0)


def now_iso() -> str:
    """UTC timestamp in the format written to `recorded`."""
    return datetime.datetime.now(datetime.timezone.utc).replace(
        microsecond=0).isoformat()


def dumps(route: Route) -> str:
    """Serialise a route.

    Written by hand rather than through yaml.safe_dump so the sample list stays
    one-per-line and reviewable in a diff, and so the header keeps its
    comments.
    """
    lat, lon, alt = route.datum
    lines = [
        '# Teach-and-repeat route. Geodetic, base_link, REP-103 yaw.',
        '#',
        '# Positions are the pose of base_link, NOT of the GNSS antenna -- see',
        '# `source` for which correction path produced them. Projected to the',
        '# map frame at load time using the datum in force then, so this file',
        '# survives a datum change; `datum` below records the one it was',
        '# recorded against.',
        'version: %d' % SCHEMA_VERSION,
        'recorded: %s' % (route.recorded or now_iso()),
        'source: %s' % route.source,
        'frame: %s' % route.frame,
        'loop: %s' % ('true' if route.loop else 'false'),
        'lane_half_width_m: %.3f' % route.lane_half_width_m,
        'datum: {latitude: %.9f, longitude: %.9f, altitude: %.3f}'
        % (lat, lon, alt),
        'samples:',
    ]
    for s in route.samples:
        lines.append(
            '  - {lat: %.9f, lon: %.9f, alt: %8.3f, yaw: %9.6f, '
            'fix: %-6s, sigma_h: %6.3f, '
            'shoulder_left_m: %.2f, shoulder_right_m: %.2f}'
            % (s.lat, s.lon, s.alt, s.yaw, s.fix, s.sigma_h,
               s.shoulder_left_m, s.shoulder_right_m))
    return '\n'.join(lines) + '\n'


def loads(text: str) -> Route:
    """Parse a route file, rejecting anything that would silently misbehave."""
    raw = yaml.safe_load(text)
    if not isinstance(raw, dict):
        raise ValueError('route file is not a mapping')

    version = raw.get('version')
    if version != SCHEMA_VERSION:
        raise ValueError('route schema version %r, expected %d'
                         % (version, SCHEMA_VERSION))

    datum = raw.get('datum') or {}
    for key in ('latitude', 'longitude'):
        if key not in datum:
            raise ValueError('route datum is missing %r' % key)

    samples = []
    for index, item in enumerate(raw.get('samples') or []):
        try:
            samples.append(Sample(
                lat=float(item['lat']),
                lon=float(item['lon']),
                alt=float(item.get('alt', 0.0)),
                yaw=float(item['yaw']),
                fix=str(item.get('fix', 'fixed')),
                sigma_h=float(item.get('sigma_h', 0.0)),
                shoulder_left_m=float(item.get('shoulder_left_m', 1.0)),
                shoulder_right_m=float(item.get('shoulder_right_m', 1.0)),
            ))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('sample %d is malformed: %s' % (index, exc))

    if len(samples) < 4:
        raise ValueError('route has %d samples; at least 4 are needed to fit '
                         'a spline' % len(samples))

    return Route(
        datum=(float(datum['latitude']), float(datum['longitude']),
               float(datum.get('altitude', 0.0))),
        loop=bool(raw.get('loop', False)),
        lane_half_width_m=float(raw.get('lane_half_width_m', 2.0)),
        source=str(raw.get('source', SOURCE_ODOMETRY)),
        frame=str(raw.get('frame', 'base_link')),
        recorded=str(raw.get('recorded', '')),
        samples=samples,
    )


def load(path: str) -> Route:
    """Read a route file from disk."""
    with open(path) as handle:
        return loads(handle.read())


def save(route: Route, path: str) -> None:
    """Write a route file to disk."""
    with open(path, 'w') as handle:
        handle.write(dumps(route))


def classify_fix(status: int, sigma_h: float,
                 fixed_sigma_m: float = 0.05,
                 float_sigma_m: float = 0.5) -> str:
    """Map a NavSatFix status + horizontal sigma onto a coarse class.

    The UM982 driver seeds the fix covariance from the reported fix quality and
    HDOP (um982_driver/src/nmea_parser.cpp), so sigma is the honest proxy for
    RTK state that survives the trip through the confidence gate -- which
    inflates covariance rather than changing status.
    """
    if status < 0:
        return 'none'
    if not math.isfinite(sigma_h) or sigma_h <= 0.0:
        return 'single'
    if sigma_h <= fixed_sigma_m:
        return 'fixed'
    if sigma_h <= float_sigma_m:
        return 'float'
    return 'single'
