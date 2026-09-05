# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Live signal aggregation for the field validation dashboard.

Collects the scattered readouts the plan tells the operator to watch --
``/gnss/fix_gated`` covariance, the raw GGA sentence, the heading quaternion,
``/route_follower/status``, the lidar -- into one :class:`Snapshot` that the
gates in :mod:`outdoor_patrol_validation.phases` consume.

Everything is stamped and everything expires. A topic that stops publishing
reads as *stale*, never as its last good value: the whole point of the
dashboard is that it does not let a dead sensor look healthy, which is exactly
what ``ros2 topic echo`` in another terminal does when the publisher dies.

Like :mod:`~outdoor_patrol_validation.phases`, this module imports no ROS.
Time comes in as a monotonic float and messages come in as plain numbers, so
the parsing and staleness rules are unit-testable without a running stack.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional

#: NMEA GGA fix quality -> the route_file fix class the plan talks about.
GGA_QUALITY_NAMES = {
    0: 'no fix',
    1: 'single',
    2: 'DGPS',
    4: 'RTK fixed',
    5: 'RTK float',
}

#: Ordered worst-to-best, matching route_file.FIX_CLASSES.
FIX_CLASSES = ('none', 'single', 'float', 'fixed')


def classify_fix(status: int, sigma_h: Optional[float],
                 fixed_sigma_m: float = 0.05,
                 float_sigma_m: float = 0.5) -> str:
    """Coarse fix class from a NavSatFix status and horizontal sigma.

    A deliberate mirror of ``route_file.classify_fix``, thresholds included.
    Phase 3 predicts what ``route_recorder`` will write into the route file,
    and the recorder classifies from status and sigma -- NOT from the GGA
    quality digit. Reading the GGA instead would be a different measurement
    that merely usually agrees.

    It also has to work where there is no GGA at all. The Gazebo sim
    synthesises a NavSatFix directly and publishes no NMEA, so a GGA-only
    implementation leaves the gate stuck on PENDING for ever there -- which
    is exactly the sort of silent no-op the dashboard exists to prevent.
    """
    if status is not None and status < 0:
        return 'none'
    if sigma_h is None or not math.isfinite(sigma_h) or sigma_h <= 0.0:
        return 'single'
    if sigma_h <= fixed_sigma_m:
        return 'fixed'
    if sigma_h <= float_sigma_m:
        return 'float'
    return 'single'


@dataclass
class Snapshot:
    """Everything the gates and the panel need, at one instant.

    ``*_ok`` flags mean "fresh enough to believe". A ``None`` value means the
    signal has never arrived at all.
    """

    t: float = 0.0

    # -- GNSS -------------------------------------------------------------
    sigma_raw: Optional[float] = None
    #: Fix class the RECORDER would write, from status + sigma. Independent
    #: of NMEA, so it works against the sim as well as the real receiver.
    fix_class: Optional[str] = None
    raw_ok: bool = False
    raw_age: float = math.inf
    sigma_gated: Optional[float] = None
    gated_ok: bool = False
    gated_age: float = math.inf
    lat: Optional[float] = None
    lon: Optional[float] = None

    gga_quality: Optional[int] = None
    gga_quality_name: str = '--'
    gga_sats: Optional[int] = None
    gga_hdop: Optional[float] = None
    gga_corr_age: Optional[float] = None
    gga_station: Optional[str] = None
    gga_ok: bool = False
    gga_age: float = math.inf

    rtcm_bps: Optional[float] = None
    rtcm_ok: bool = False
    rtcm_age: float = math.inf

    # -- attitude ---------------------------------------------------------
    #: ENU degrees: 0 = East, 90 = North.
    heading_yaw_deg: Optional[float] = None
    heading_ok: bool = False
    heading_age: float = math.inf

    # -- pose -------------------------------------------------------------
    odom_x: Optional[float] = None
    odom_y: Optional[float] = None
    odom_yaw_deg: Optional[float] = None
    odom_speed: Optional[float] = None
    odom_ok: bool = False
    odom_age: float = math.inf
    tf_ok: bool = False

    #: Antenna position minus base_link, rotated into body axes. Phase 4.
    lever_x: Optional[float] = None
    lever_y: Optional[float] = None

    # -- follower ---------------------------------------------------------
    state: Optional[str] = None
    s: Optional[float] = None
    travelled: Optional[float] = None
    cross_track: Optional[float] = None
    d_cmd: Optional[float] = None
    d_target: Optional[float] = None
    blocked: Optional[list] = None
    follower_speed: Optional[float] = None
    follower_omega: Optional[float] = None
    sigma_h: Optional[float] = None
    #: Follower tunables, echoed back in its status so the panel can show
    #: what is actually in force rather than what was last typed.
    avoidance: Optional[bool] = None
    corridor_half_width_m: Optional[float] = None
    nominal_speed_ms: Optional[float] = None
    show_corridor: Optional[bool] = None
    follower_ok: bool = False
    follower_age: float = math.inf

    # -- lidar ------------------------------------------------------------
    scan_front_min: Optional[float] = None
    scan_left_min: Optional[float] = None
    scan_right_min: Optional[float] = None
    scan_ok: bool = False
    scan_age: float = math.inf

    # -- chassis ----------------------------------------------------------
    cmd_speed: Optional[float] = None
    cmd_omega: Optional[float] = None
    cmd_ok: bool = False

    #: topic name -> publishing within its timeout.
    topics: Dict[str, bool] = field(default_factory=dict)

    def as_dict(self) -> dict:
        """Flat, JSON-safe view for the panel."""
        def r(v, n=3):
            return None if v is None else round(v, n)
        return {
            'sigma_raw': r(self.sigma_raw, 4),
            'sigma_gated': r(self.sigma_gated, 4),
            'raw_ok': self.raw_ok,
            'gated_ok': self.gated_ok,
            'fix_age': None if math.isinf(self.raw_age) else r(self.raw_age, 1),
            'fix_class': self.fix_class,
            'lat': r(self.lat, 8),
            'lon': r(self.lon, 8),
            'gga_quality': self.gga_quality,
            'gga_quality_name': self.gga_quality_name,
            'gga_sats': self.gga_sats,
            'gga_hdop': r(self.gga_hdop, 2),
            'gga_corr_age': r(self.gga_corr_age, 1),
            'gga_station': self.gga_station,
            'gga_ok': self.gga_ok,
            'rtcm_bps': r(self.rtcm_bps, 0),
            'rtcm_ok': self.rtcm_ok,
            'heading_yaw_deg': r(self.heading_yaw_deg, 1),
            'heading_ok': self.heading_ok,
            'odom_x': r(self.odom_x, 2),
            'odom_y': r(self.odom_y, 2),
            'odom_yaw_deg': r(self.odom_yaw_deg, 1),
            'odom_speed': r(self.odom_speed, 2),
            'odom_ok': self.odom_ok,
            'tf_ok': self.tf_ok,
            'lever_x': r(self.lever_x),
            'lever_y': r(self.lever_y),
            'state': self.state,
            's': r(self.s, 1),
            'travelled': r(self.travelled, 1),
            'cross_track': r(self.cross_track),
            'd_cmd': r(self.d_cmd),
            'blocked': self.blocked,
            'follower_speed': r(self.follower_speed),
            'sigma_h': r(self.sigma_h, 4),
            'avoidance': self.avoidance,
            'follower_ok': self.follower_ok,
            'scan_front_min': r(self.scan_front_min, 2),
            'scan_left_min': r(self.scan_left_min, 2),
            'scan_right_min': r(self.scan_right_min, 2),
            'scan_ok': self.scan_ok,
            'cmd_speed': r(self.cmd_speed),
            'cmd_omega': r(self.cmd_omega),
            'topics': self.topics,
        }


class _Slot:
    """A value with an expiry date."""

    __slots__ = ('value', 'stamp', 'timeout')

    def __init__(self, timeout: float) -> None:
        self.value = None
        self.stamp: Optional[float] = None
        self.timeout = timeout

    def set(self, value, now: float) -> None:
        self.value = value
        self.stamp = now

    def age(self, now: float) -> float:
        return math.inf if self.stamp is None else now - self.stamp

    def ok(self, now: float) -> bool:
        return self.age(now) <= self.timeout

    def get(self, now: float):
        """The value, or ``None`` once it has gone stale."""
        return self.value if self.ok(now) else None


def yaw_deg_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """ENU yaw in degrees, 0 = East, 90 = North."""
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def sigma_from_covariance(cov) -> Optional[float]:
    """Horizontal 1-sigma from a NavSatFix ``position_covariance``.

    ``cov[0]`` is the east variance, so sigma is its square root. This is the
    number ``confidence_gate`` compares against ``max_horizontal_sigma_m`` and
    the number the plan's soak table is written in.
    """
    if cov is None or len(cov) < 1:
        return None
    var = float(cov[0])
    if var < 0.0 or not math.isfinite(var):
        return None
    return math.sqrt(var)


def parse_gga(sentence: str) -> Optional[dict]:
    """Pull quality, satellites, HDOP, correction age and station from a GGA.

    The plan reads these straight off the wire because they answer questions
    the NavSatFix cannot: *which* base is being used, and whether the
    correction stream has stalled while the socket stayed open.

    Returns ``None`` for anything that is not a GGA, so it can be fed the
    whole ``/um982_driver/nmea_sentence`` stream.
    """
    if not sentence:
        return None
    text = sentence.strip()
    if '*' in text:
        text = text.split('*', 1)[0]
    parts = text.split(',')
    if not parts or not parts[0].endswith('GGA'):
        return None
    if len(parts) < 15:
        return None

    def num(token, cast=float):
        token = token.strip()
        if not token:
            return None
        try:
            return cast(token)
        except ValueError:
            return None

    quality = num(parts[6], int)
    return {
        'quality': quality,
        'quality_name': GGA_QUALITY_NAMES.get(quality, f'q{quality}'),
        'sats': num(parts[7], int),
        'hdop': num(parts[8]),
        'corr_age': num(parts[13]),
        'station': parts[14].strip() or None,
    }


def sector_min(ranges: List[float], angle_min: float, angle_increment: float,
               centre_deg: float, half_width_deg: float,
               range_min: float = 0.06,
               range_max: float = 25.0) -> Optional[float]:
    """Smallest valid return in a sector of the *raw* scan frame.

    ``centre_deg`` is a bearing in the scan's own angles, not in body axes.
    Callers wanting "ahead" must add the lidar's mounting offset first -- see
    :attr:`Signals.forward_offset_deg`.
    """
    if not ranges or angle_increment == 0.0:
        return None
    best: Optional[float] = None
    centre = math.radians(centre_deg)
    half = math.radians(half_width_deg)
    for i, value in enumerate(ranges):
        if value is None or not math.isfinite(value):
            continue
        if value < range_min or value > range_max:
            continue
        angle = angle_min + i * angle_increment
        if abs((angle - centre + math.pi) % (2.0 * math.pi) - math.pi) > half:
            continue
        if best is None or value < best:
            best = value
    return best


class Signals:
    """Mutable store of the latest reading from every topic.

    ROS callbacks push in through the ``on_*`` methods; :meth:`snapshot`
    renders a consistent, expiry-applied view for the gates.
    """

    #: Topics Phase 0 insists on before anything else may run.
    REQUIRED = ('fix', 'fix_gated', 'heading', 'odometry', 'scan')

    def __init__(self, timeouts: Optional[Dict[str, float]] = None,
                 forward_offset_deg: float = 180.0) -> None:
        #: Bearing of ROBOT-FORWARD in the raw scan frame, degrees.
        #:
        #: The RPLIDAR C1 is mounted yaw-180 -- its 0 deg points at the robot
        #: body behind it -- so robot-forward is at 180 deg in ``/scan``. This
        #: node reads raw scan angles rather than transforming through TF, so
        #: it MUST match the ``lidar_link`` yaw in ``chassis.yaml``, exactly
        #: as ``scan_safety``'s ``forward_offset_deg`` does.
        #:
        #: Getting this wrong is quiet and nasty: the front and rear sectors
        #: swap, so "never closer than 0.3 m ahead" would be watching behind.
        self.forward_offset_deg = forward_offset_deg
        t = dict(timeouts or {})
        self.raw = _Slot(t.get('fix', 2.0))
        self.gated = _Slot(t.get('fix_gated', 2.0))
        self.gga = _Slot(t.get('gga', 5.0))
        self.rtcm = _Slot(t.get('rtcm', 10.0))
        self.heading = _Slot(t.get('heading', 2.0))
        self.odom = _Slot(t.get('odometry', 2.0))
        self.follower = _Slot(t.get('follower', 2.0))
        self.scan = _Slot(t.get('scan', 2.0))
        self.cmd = _Slot(t.get('cmd', 2.0))
        self.lever = _Slot(t.get('lever', 5.0))
        self.tf_ok = False
        self._rtcm_bytes = 0
        self._rtcm_window_start: Optional[float] = None
        self._rtcm_bps: Optional[float] = None

    # -- ingest ------------------------------------------------------------

    def on_fix(self, lat, lon, covariance, now: float, status: int = 0) -> None:
        sigma = sigma_from_covariance(covariance)
        self.raw.set({'lat': lat, 'lon': lon, 'sigma': sigma,
                      'fix_class': classify_fix(status, sigma)}, now)

    def on_fix_gated(self, covariance, now: float) -> None:
        self.gated.set({'sigma': sigma_from_covariance(covariance)}, now)

    def on_nmea(self, sentence: str, now: float) -> None:
        parsed = parse_gga(sentence)
        if parsed is not None:
            self.gga.set(parsed, now)

    def on_rtcm(self, length: int, now: float) -> None:
        """Track correction throughput, not just liveness.

        A caster that holds the socket open but sends nothing is the failure
        mode the plan describes, and it is invisible if you only check that
        the topic exists.
        """
        if self._rtcm_window_start is None:
            self._rtcm_window_start = now
        self._rtcm_bytes += int(length)
        span = now - self._rtcm_window_start
        if span >= 5.0:
            self._rtcm_bps = self._rtcm_bytes / span
            self._rtcm_bytes = 0
            self._rtcm_window_start = now
        self.rtcm.set(True, now)

    def on_heading(self, x, y, z, w, now: float) -> None:
        self.heading.set(yaw_deg_from_quaternion(x, y, z, w), now)

    def on_odometry(self, x, y, quat, speed, now: float) -> None:
        self.odom.set({
            'x': x, 'y': y,
            'yaw': yaw_deg_from_quaternion(*quat),
            'speed': speed,
        }, now)

    def on_follower_status(self, status: dict, now: float) -> None:
        self.follower.set(dict(status), now)

    def on_scan(self, ranges, angle_min, angle_increment, now: float,
                range_min: float = 0.06, range_max: float = 25.0) -> None:
        # Body-frame bearings, converted into the scan's own angles.
        ahead = self.forward_offset_deg
        self.scan.set({
            'front': sector_min(ranges, angle_min, angle_increment,
                                ahead, 30.0, range_min, range_max),
            'left': sector_min(ranges, angle_min, angle_increment,
                               ahead + 90.0, 30.0, range_min, range_max),
            'right': sector_min(ranges, angle_min, angle_increment,
                                ahead - 90.0, 30.0, range_min, range_max),
        }, now)

    def on_cmd_vel(self, linear, angular, now: float) -> None:
        self.cmd.set({'v': linear, 'w': angular}, now)

    def on_lever_arm(self, dx: float, dy: float, now: float) -> None:
        """Antenna offset from base_link, already rotated into body axes."""
        self.lever.set({'x': dx, 'y': dy}, now)

    def on_tf(self, ok: bool) -> None:
        self.tf_ok = ok

    # -- render ------------------------------------------------------------

    def snapshot(self, now: float) -> Snapshot:
        snap = Snapshot(t=now)

        raw = self.raw.get(now)
        snap.raw_ok = self.raw.ok(now)
        snap.raw_age = self.raw.age(now)
        if raw:
            snap.sigma_raw = raw['sigma']
            snap.lat, snap.lon = raw['lat'], raw['lon']
            snap.fix_class = raw['fix_class']

        gated = self.gated.get(now)
        snap.gated_ok = self.gated.ok(now)
        snap.gated_age = self.gated.age(now)
        if gated:
            snap.sigma_gated = gated['sigma']

        gga = self.gga.get(now)
        snap.gga_ok = self.gga.ok(now)
        snap.gga_age = self.gga.age(now)
        if gga:
            snap.gga_quality = gga['quality']
            snap.gga_quality_name = gga['quality_name']
            snap.gga_sats = gga['sats']
            snap.gga_hdop = gga['hdop']
            snap.gga_corr_age = gga['corr_age']
            snap.gga_station = gga['station']

        snap.rtcm_ok = self.rtcm.ok(now)
        snap.rtcm_age = self.rtcm.age(now)
        snap.rtcm_bps = self._rtcm_bps if snap.rtcm_ok else None

        snap.heading_ok = self.heading.ok(now)
        snap.heading_age = self.heading.age(now)
        snap.heading_yaw_deg = self.heading.get(now)

        odom = self.odom.get(now)
        snap.odom_ok = self.odom.ok(now)
        snap.odom_age = self.odom.age(now)
        if odom:
            snap.odom_x, snap.odom_y = odom['x'], odom['y']
            snap.odom_yaw_deg, snap.odom_speed = odom['yaw'], odom['speed']

        lever = self.lever.get(now)
        if lever:
            snap.lever_x, snap.lever_y = lever['x'], lever['y']

        status = self.follower.get(now)
        snap.follower_ok = self.follower.ok(now)
        snap.follower_age = self.follower.age(now)
        if status:
            snap.state = status.get('state')
            snap.s = status.get('s')
            snap.travelled = status.get('travelled')
            snap.cross_track = status.get('cross_track')
            snap.d_cmd = status.get('d_cmd')
            snap.d_target = status.get('d_target')
            snap.blocked = status.get('blocked')
            snap.follower_speed = status.get('speed')
            snap.follower_omega = status.get('omega')
            snap.sigma_h = status.get('sigma_h')
            snap.avoidance = status.get('avoidance')
            snap.corridor_half_width_m = status.get('corridor_half_width_m')
            snap.nominal_speed_ms = status.get('nominal_speed_ms')
            snap.show_corridor = status.get('show_corridor')

        scan = self.scan.get(now)
        snap.scan_ok = self.scan.ok(now)
        snap.scan_age = self.scan.age(now)
        if scan:
            snap.scan_front_min = scan['front']
            snap.scan_left_min = scan['left']
            snap.scan_right_min = scan['right']

        cmd = self.cmd.get(now)
        snap.cmd_ok = self.cmd.ok(now)
        if cmd:
            snap.cmd_speed, snap.cmd_omega = cmd['v'], cmd['w']

        snap.tf_ok = self.tf_ok
        snap.topics = {
            'fix': self.raw.ok(now),
            'fix_gated': self.gated.ok(now),
            'heading': self.heading.ok(now),
            'odometry': self.odom.ok(now),
            'scan': self.scan.ok(now),
        }
        return snap
