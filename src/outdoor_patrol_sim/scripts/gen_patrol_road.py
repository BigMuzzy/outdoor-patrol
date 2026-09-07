#!/usr/bin/env python3
# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Generate the patrol-road test world and its ground-truth centerline.

One definition, three artefacts:

  worlds/patrol_road.sdf             clean road, no obstacles
  worlds/patrol_road_obstacles.sdf   same road + three barriers
  worlds/patrol_road_centerline.yaml ground truth for the driver and scorer

Keeping the world and the centerline on the same generator is the point: the
teach-pass driver, the route scorer and the rendered road can never disagree
about where the road is, because none of them owns the number.

The road is a **rounded square loop** -- four straights joined by four
quarter-circle corners -- sized so the total centerline length is exactly the
requested value (default 100 m)::

    4 * straight + 2 * pi * corner_radius = length

Corners are rounded rather than square so a teach pass can be driven at speed
without in-place pivots, which the recorder's yaw-triggered sampling would
otherwise have to resolve from a near-stationary heading.

Cross-section (default): a 4 m lane (+/- 2 m of the centerline) with a 1 m
shoulder on each side, giving a 6 m corridor. Lateral offsets follow REP-103:
**+ is left, - is right**, so the right shoulder is at negative offsets.

The road carries NO collision geometry. It is paint, not kerb -- a collision
box at the road edge would put a return into /scan and trip the forward brake
on the road surface itself.

Obstacles are barriers wide enough to block the lane AND the left shoulder,
leaving a passable gap on the right shoulder only. That makes "it went around
on the right" a measurable fact rather than a judgement call.

Usage::

    python3 scripts/gen_patrol_road.py            # writes into worlds/
    python3 scripts/gen_patrol_road.py --check    # verify committed files
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field

# Datum of the existing patrol_yard world. Shared so a sim route recorded in
# one world is directly comparable in the other, and so navsat_transform can be
# pinned to a FIXED datum (wait_for_datum: true) instead of drifting to
# wherever the first fix lands.
DEFAULT_DATUM = (-41.286460, 174.776236, 10.0)

# Barrier stations, in metres along the centerline from the start point.
# Chosen to cover the three cases that fail differently:
#   8 m  - clean straight, the easy case
#   46 m - mid-corner, where path curvature and the lateral offset interact
#   77 m - 2 m after a corner exit, while the follower is still settling
# Spacing is >= 30 m so the "resumed within 10 m" assertion is never
# contaminated by the next barrier entering the look-ahead window.
DEFAULT_OBSTACLE_STATIONS = (8.0, 46.0, 77.0)

# Barrier footprint: thickness along the path, span across it, height.
OBSTACLE_SIZE = (0.4, 5.0, 1.0)


@dataclass(frozen=True)
class RoadSpec:
    """Everything the road geometry is derived from."""

    length_m: float = 100.0
    corner_radius_m: float = 5.0
    lane_half_width_m: float = 2.0
    shoulder_width_m: float = 1.0
    sample_spacing_m: float = 0.25
    datum: tuple = DEFAULT_DATUM
    obstacle_stations: tuple = DEFAULT_OBSTACLE_STATIONS

    @property
    def straight_m(self) -> float:
        """Length of each of the four straights."""
        return (self.length_m - 2.0 * math.pi * self.corner_radius_m) / 4.0

    @property
    def corridor_half_width_m(self) -> float:
        return self.lane_half_width_m + self.shoulder_width_m

    def validate(self) -> None:
        if self.straight_m <= 0.0:
            raise ValueError(
                'corner_radius %.2f m is too large for a %.1f m loop: the '
                'four corners alone are %.2f m'
                % (self.corner_radius_m, self.length_m,
                   2.0 * math.pi * self.corner_radius_m))
        if self.shoulder_width_m <= 0.0:
            raise ValueError('shoulder_width_m must be positive')


@dataclass
class Pose2D:
    s: float
    x: float
    y: float
    yaw: float

    def offset(self, lateral: float) -> tuple:
        """Point `lateral` metres to the LEFT of this pose (- is right)."""
        return (self.x - lateral * math.sin(self.yaw),
                self.y + lateral * math.cos(self.yaw))


@dataclass
class Segment:
    """One straight or one arc of the loop, parameterised by arc length."""

    kind: str          # 'straight' | 'arc'
    s0: float
    length: float
    # straight: start point + heading. arc: centre + start angle + radius.
    x0: float = 0.0
    y0: float = 0.0
    yaw0: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    theta0: float = 0.0
    radius: float = 0.0

    @property
    def s1(self) -> float:
        return self.s0 + self.length

    def pose_at(self, s: float) -> Pose2D:
        t = s - self.s0
        if self.kind == 'straight':
            return Pose2D(s,
                          self.x0 + t * math.cos(self.yaw0),
                          self.y0 + t * math.sin(self.yaw0),
                          self.yaw0)
        # Arcs are all traversed counter-clockwise, so the heading leads the
        # radius vector by +90 degrees.
        theta = self.theta0 + t / self.radius
        return Pose2D(s,
                      self.cx + self.radius * math.cos(theta),
                      self.cy + self.radius * math.sin(theta),
                      _wrap(theta + math.pi / 2.0))


@dataclass
class Road:
    spec: RoadSpec
    segments: list = field(default_factory=list)

    def pose_at(self, s: float) -> Pose2D:
        s = s % self.spec.length_m
        for seg in self.segments:
            if s < seg.s1 or seg is self.segments[-1]:
                return seg.pose_at(s)
        raise AssertionError('unreachable')

    def samples(self) -> list:
        """Centerline poses at the configured spacing, s=0 .. s=length."""
        spacing = self.spec.sample_spacing_m
        count = int(round(self.spec.length_m / spacing))
        return [self.pose_at(i * spacing) for i in range(count + 1)]


def _wrap(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def build_road(spec: RoadSpec) -> Road:
    """Lay out the rounded square, counter-clockwise from the south straight.

    Straights sit at x = +/-a and y = +/-a with a = straight/2 + radius; the
    four corner centres sit at (+/-straight/2, +/-straight/2). s = 0 is the
    west end of the south straight, heading east.
    """
    spec.validate()
    ls = spec.straight_m
    r = spec.corner_radius_m
    half = ls / 2.0
    a = half + r
    quarter = math.pi / 2.0 * r

    # (straight start, straight heading, following corner centre, arc start)
    plan = [
        ((-half, -a), 0.0, (half, -half), -math.pi / 2.0),
        ((a, -half), math.pi / 2.0, (half, half), 0.0),
        ((half, a), math.pi, (-half, half), math.pi / 2.0),
        ((-a, half), -math.pi / 2.0, (-half, -half), math.pi),
    ]

    segments = []
    s = 0.0
    for (sx, sy), heading, (cx, cy), theta0 in plan:
        segments.append(Segment('straight', s, ls,
                                x0=sx, y0=sy, yaw0=_wrap(heading)))
        s += ls
        segments.append(Segment('arc', s, quarter,
                                cx=cx, cy=cy, theta0=theta0, radius=r))
        s += quarter

    road = Road(spec, segments)
    _self_check(road)
    return road


def _self_check(road: Road) -> None:
    """Fail loudly if the segments do not actually join up."""
    total = sum(seg.length for seg in road.segments)
    if abs(total - road.spec.length_m) > 1e-6:
        raise AssertionError('segment lengths sum to %.6f, want %.6f'
                             % (total, road.spec.length_m))
    for prev, nxt in zip(road.segments, road.segments[1:] + road.segments[:1]):
        end = prev.pose_at(prev.s1)
        start = nxt.pose_at(nxt.s0)
        gap = math.hypot(end.x - start.x, end.y - start.y)
        kink = abs(_wrap(end.yaw - start.yaw))
        if gap > 1e-9 or kink > 1e-9:
            raise AssertionError(
                'discontinuity between %s@%.3f and %s@%.3f: gap %.9f m, '
                'kink %.9f rad' % (prev.kind, prev.s0, nxt.kind, nxt.s0,
                                   gap, kink))


def obstacle_poses(road: Road) -> list:
    """World poses for the barriers, plus the gap they leave on the right.

    Each barrier is placed so its RIGHT edge stops short of the outer shoulder
    edge, and its LEFT edge overhangs the left shoulder edge. Only the right
    shoulder is passable.
    """
    spec = road.spec
    thickness, span, height = OBSTACLE_SIZE
    # Overhang the left edge past the corridor so no left gap exists at all.
    left_edge = spec.corridor_half_width_m + 0.2
    right_edge = left_edge - span
    centre_lateral = (left_edge + right_edge) / 2.0
    gap_width = right_edge + spec.corridor_half_width_m

    out = []
    for index, station in enumerate(spec.obstacle_stations, start=1):
        pose = road.pose_at(station)
        x, y = pose.offset(centre_lateral)
        out.append({
            'name': 'barrier_%d' % index,
            's_m': station,
            'lateral_offset_m': centre_lateral,
            'right_edge_offset_m': right_edge,
            'left_edge_offset_m': left_edge,
            'gap_side': 'right',
            'gap_width_m': gap_width,
            'gap_center_offset_m': (right_edge
                                    - spec.corridor_half_width_m) / 2.0,
            'size_m': [thickness, span, height],
            'x': x,
            'y': y,
            'z': height / 2.0,
            'yaw': pose.yaw,
        })
    return out


# --------------------------------------------------------------------------
# YAML emission (hand-rolled: keeps the 400-sample list one-per-line readable
# and avoids a PyYAML dependency in a plain build script)
# --------------------------------------------------------------------------

def centerline_yaml(road: Road) -> str:
    spec = road.spec
    lat, lon, alt = spec.datum
    lines = [
        '# GENERATED by scripts/gen_patrol_road.py -- do not edit by hand.',
        '#',
        '# Ground-truth centerline of the patrol road world. Consumed by the',
        '# sim teach-pass driver and by the route scorer; the matching world',
        '# is worlds/patrol_road.sdf (or patrol_road_obstacles.sdf).',
        '#',
        '# Frame: Gazebo world / ROS `map`, ENU, metres. Lateral offsets are',
        '# REP-103 signed: + left, - right.',
        'road:',
        '  length_m: %.6f' % spec.length_m,
        '  loop: true',
        '  corner_radius_m: %.6f' % spec.corner_radius_m,
        '  straight_m: %.6f' % spec.straight_m,
        '  lane_half_width_m: %.6f' % spec.lane_half_width_m,
        '  shoulder_width_m: %.6f' % spec.shoulder_width_m,
        '  corridor_half_width_m: %.6f' % spec.corridor_half_width_m,
        '  sample_spacing_m: %.6f' % spec.sample_spacing_m,
        '  datum: {latitude: %.8f, longitude: %.8f, altitude: %.3f}'
        % (lat, lon, alt),
        '  obstacles:',
    ]
    for obs in obstacle_poses(road):
        lines.append('    - name: %s' % obs['name'])
        lines.append('      s_m: %.3f' % obs['s_m'])
        lines.append('      x: %.6f' % obs['x'])
        lines.append('      y: %.6f' % obs['y'])
        lines.append('      yaw: %.6f' % obs['yaw'])
        lines.append('      size_m: [%.3f, %.3f, %.3f]' % tuple(obs['size_m']))
        lines.append('      lateral_offset_m: %.3f' % obs['lateral_offset_m'])
        lines.append('      right_edge_offset_m: %.3f'
                     % obs['right_edge_offset_m'])
        lines.append('      left_edge_offset_m: %.3f'
                     % obs['left_edge_offset_m'])
        lines.append('      gap_side: %s' % obs['gap_side'])
        lines.append('      gap_width_m: %.3f' % obs['gap_width_m'])
        lines.append('      gap_center_offset_m: %.3f'
                     % obs['gap_center_offset_m'])
    lines.append('  samples:')
    for pose in road.samples():
        lines.append('    - {s: %8.3f, x: %10.4f, y: %10.4f, yaw: %9.6f}'
                     % (pose.s, pose.x, pose.y, pose.yaw))
    return '\n'.join(lines) + '\n'


# --------------------------------------------------------------------------
# SDF emission
# --------------------------------------------------------------------------

_WORLD_HEAD = """<?xml version="1.0" ?>
<!--
  GENERATED by scripts/gen_patrol_road.py -- do not edit by hand.

  {title}

  A {length:.0f} m rounded-square patrol road: {lane:.0f} m lane
  (+/-{half:.1f} m of the centerline) with a {shoulder:.0f} m shoulder each
  side, corner radius {radius:.1f} m. The ground-truth centerline for this
  world is worlds/patrol_road_centerline.yaml.

  The road surface is VISUAL ONLY. Giving it collision geometry would put
  returns into /scan and trip the forward brake on the road itself.
-->
<sdf version="1.9">
  <world name="{world_name}">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <!-- Renders the gpu_lidar; needs a GL 3.3 context (llvmpipe is enough). -->
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system"
            name="gz::sim::systems::NavSat"/>

    <!-- Datum for the simulated GNSS. Matches patrol_yard.sdf, and is the
         value to pin navsat_transform's `datum` to for sim runs. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat:.6f}</latitude_deg>
      <longitude_deg>{lon:.6f}</longitude_deg>
      <elevation>{alt:.1f}</elevation>
    </spherical_coordinates>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.35 0.4 0.3 1</ambient>
            <diffuse>0.4 0.5 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

_WORLD_TAIL = """
  </world>
</sdf>
"""


def _road_bands(spec: RoadSpec) -> list:
    """(name, lateral centre, width, z, rgb) for each painted band."""
    lane = spec.lane_half_width_m
    shoulder = spec.shoulder_width_m
    edge = lane + shoulder / 2.0
    return [
        ('lane', 0.0, 2.0 * lane, 0.005, (0.20, 0.20, 0.21)),
        ('shoulder_left', edge, shoulder, 0.004, (0.45, 0.42, 0.32)),
        ('shoulder_right', -edge, shoulder, 0.004, (0.45, 0.42, 0.32)),
        ('edge_line_left', lane, 0.12, 0.010, (0.85, 0.85, 0.82)),
        ('edge_line_right', -lane, 0.12, 0.010, (0.85, 0.85, 0.82)),
    ]


def _tile_stations(road: Road) -> list:
    """(s_mid, tile_length) pairs covering the loop.

    Straights get one tile each; arcs are chopped into ~1 m tiles so the
    chord error stays invisible.
    """
    out = []
    for seg in road.segments:
        if seg.kind == 'straight':
            out.append((seg.s0 + seg.length / 2.0, seg.length))
            continue
        count = max(1, int(round(seg.length / 1.0)))
        step = seg.length / count
        for i in range(count):
            out.append((seg.s0 + (i + 0.5) * step, step))
    return out


def road_model_sdf(road: Road) -> str:
    """One static, collision-free model holding every painted tile."""
    spec = road.spec
    tiles = _tile_stations(road)
    parts = ['    <!-- Painted road surface: visual only, no collision. -->',
             '    <model name="patrol_road">',
             '      <static>true</static>',
             '      <link name="surface">']
    for band, lateral, width, z, rgb in _road_bands(spec):
        for index, (s_mid, tile_len) in enumerate(tiles):
            pose = road.pose_at(s_mid)
            x, y = pose.offset(lateral)
            # Widen each tile slightly so neighbours overlap rather than
            # leaving hairline gaps on the arcs.
            length = tile_len * 1.08 + 0.05
            parts.append(
                '        <visual name="%s_%03d">\n'
                '          <pose>%.4f %.4f %.4f 0 0 %.6f</pose>\n'
                '          <geometry><box><size>%.4f %.4f 0.002</size>'
                '</box></geometry>\n'
                '          <material>\n'
                '            <ambient>%.2f %.2f %.2f 1</ambient>\n'
                '            <diffuse>%.2f %.2f %.2f 1</diffuse>\n'
                '          </material>\n'
                '        </visual>'
                % (band, index, x, y, z, pose.yaw, length, width,
                   rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]))
    parts += ['      </link>', '    </model>']
    return '\n'.join(parts) + '\n'


def obstacles_sdf(road: Road) -> str:
    parts = ['',
             '    <!-- Lane barriers. Each spans the lane and the LEFT',
             '         shoulder, leaving a passable gap on the RIGHT shoulder',
             '         only, so a left-side dodge is geometrically',
             '         impossible. -->']
    for obs in obstacle_poses(road):
        sx, sy, sz = obs['size_m']
        parts.append(
            '    <model name="%s">\n'
            '      <static>true</static>\n'
            '      <pose>%.4f %.4f %.4f 0 0 %.6f</pose>\n'
            '      <link name="link">\n'
            '        <collision name="collision">\n'
            '          <geometry><box><size>%.3f %.3f %.3f</size></box>'
            '</geometry>\n'
            '        </collision>\n'
            '        <visual name="visual">\n'
            '          <geometry><box><size>%.3f %.3f %.3f</size></box>'
            '</geometry>\n'
            '          <material>\n'
            '            <ambient>0.7 0.3 0.2 1</ambient>\n'
            '            <diffuse>0.8 0.35 0.25 1</diffuse>\n'
            '          </material>\n'
            '        </visual>\n'
            '      </link>\n'
            '    </model>'
            % (obs['name'], obs['x'], obs['y'], obs['z'], obs['yaw'],
               sx, sy, sz, sx, sy, sz))
    return '\n'.join(parts) + '\n'


def world_sdf(road: Road, with_obstacles: bool) -> str:
    spec = road.spec
    lat, lon, alt = spec.datum
    head = _WORLD_HEAD.format(
        title=('Patrol road with three lane barriers.' if with_obstacles
               else 'Patrol road, clear of obstacles.'),
        world_name=('patrol_road_obstacles' if with_obstacles
                    else 'patrol_road'),
        length=spec.length_m,
        lane=2.0 * spec.lane_half_width_m,
        half=spec.lane_half_width_m,
        shoulder=spec.shoulder_width_m,
        radius=spec.corner_radius_m,
        lat=lat, lon=lon, alt=alt)
    body = road_model_sdf(road)
    if with_obstacles:
        body += obstacles_sdf(road)
    return head + '\n' + body + _WORLD_TAIL


# --------------------------------------------------------------------------

def artefacts(road: Road, prefix: str = 'patrol_road') -> dict:
    return {
        '%s.sdf' % prefix: world_sdf(road, with_obstacles=False),
        '%s_obstacles.sdf' % prefix: world_sdf(road, with_obstacles=True),
        '%s_centerline.yaml' % prefix: centerline_yaml(road),
    }


def main(argv=None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    default_out = os.path.normpath(os.path.join(here, os.pardir, 'worlds'))

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out-dir', default=default_out)
    parser.add_argument('--length', type=float, default=100.0,
                        help='total centerline length, metres')
    parser.add_argument('--corner-radius', type=float, default=5.0)
    parser.add_argument('--lane-half-width', type=float, default=2.0)
    parser.add_argument('--shoulder-width', type=float, default=1.0)
    parser.add_argument('--sample-spacing', type=float, default=0.25)
    parser.add_argument('--name-prefix', default='patrol_road',
                        help='basename of the generated artefacts, so a '
                             'second road (e.g. a driveway-sized loop) can '
                             'live beside the default one')
    parser.add_argument('--check', action='store_true',
                        help='exit non-zero if the files on disk differ from '
                             'what would be generated')
    args = parser.parse_args(argv)

    spec = RoadSpec(length_m=args.length,
                    corner_radius_m=args.corner_radius,
                    lane_half_width_m=args.lane_half_width,
                    shoulder_width_m=args.shoulder_width,
                    sample_spacing_m=args.sample_spacing)
    road = build_road(spec)
    files = artefacts(road, args.name_prefix)

    if args.check:
        stale = []
        for name, text in files.items():
            path = os.path.join(args.out_dir, name)
            if not os.path.exists(path):
                stale.append('%s (missing)' % name)
            elif open(path).read() != text:
                stale.append('%s (out of date)' % name)
        if stale:
            print('stale generated files: %s' % ', '.join(stale),
                  file=sys.stderr)
            print('re-run: python3 scripts/gen_patrol_road.py',
                  file=sys.stderr)
            return 1
        print('generated road artefacts are up to date')
        return 0

    os.makedirs(args.out_dir, exist_ok=True)
    for name, text in files.items():
        path = os.path.join(args.out_dir, name)
        with open(path, 'w') as handle:
            handle.write(text)
        print('wrote %s (%d bytes)' % (path, len(text)))

    print('road: %.1f m loop, %.3f m straights, %.1f m corners, '
          '%.1f m lane + %.1f m shoulders'
          % (spec.length_m, spec.straight_m, spec.corner_radius_m,
             2 * spec.lane_half_width_m, spec.shoulder_width_m))
    for obs in obstacle_poses(road):
        print('  %s at s=%.1f m -> %.2f m gap on the %s shoulder'
              % (obs['name'], obs['s_m'], obs['gap_width_m'],
                 obs['gap_side']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
