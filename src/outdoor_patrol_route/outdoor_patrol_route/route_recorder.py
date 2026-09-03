# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Record one manually driven pass as a geodetic, base_link route file.

Records the pose of **base_link**, not of the GNSS antenna. On this vehicle
``gnss_link`` sits 0.28 m forward and 0.42 m right of ``base_link``
(outdoor_patrol_bringup/config/chassis.yaml), so an uncorrected recording puts
the "centerline" 0.42 m to the right of where the robot centre actually
travelled -- and through a corner the antenna sweeps an arc while ``base_link``
barely translates, which no constant offset can describe.

Three sources, selected by the ``source`` parameter. The first two both
produce a base_link route by independent means; the third exists so the
correction can be *measured* rather than asserted.

``odometry_global`` (default)
    Pose straight off ``/odometry/global``, which is already a ``base_link``
    pose: the driver stamps the fix ``frame_id: gnss_link``
    (um982_driver_node.cpp), ``confidence_gate`` passes the frame through, and
    ``navsat_transform`` applies the ``gnss_link -> base_link`` lever arm from
    TF. EKF-smoothed, so it does not carry the raw per-fix noise. Map XY is
    buffered during the run and converted to geodetic in one batch at save
    time through ``navsat_transform``'s ``toLL`` service, which keeps the
    projection and the datum consistent by construction.

``fix_lever_arm``
    The gated fix -- i.e. the antenna -- with the lever arm subtracted
    explicitly, rotating the TF antenna offset by the current yaw. Touches
    none of ``navsat_transform``'s internals and needs no service, so it is
    the independent cross-check when the primary path is under suspicion.

``raw_antenna``
    The gated fix, uncorrected. **Not** a base_link route and refused by the
    follower. Its only job is to be the control in the differential test: a
    correction that is not doing anything scores the same as this file, and a
    correction that is doing its job scores ~0.42 m better.

Sampling triggers on **1 m of travel or 5 degrees of yaw change**, not on a
timer: a timer under-samples corners and over-samples straights.

Recording runs from start-up. Call ``~/save`` (std_srvs/Trigger) to write the
file; ``~/discard`` throws the buffer away and starts over.
"""

from __future__ import annotations

import math
import os
import time

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_localization.srv import ToLL
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from outdoor_patrol_route import route_file
from outdoor_patrol_route.route_file import Route, Sample

# WGS84 mean radius. Only ever used over sub-metre spans (the lever arm) and
# for the sampling trigger, where a spherical approximation is orders of
# magnitude inside the noise.
_EARTH_RADIUS_M = 6371000.0


def _metres_per_degree(latitude: float):
    """(north, east) metres per degree of latitude / longitude."""
    per_lat = _EARTH_RADIUS_M * math.pi / 180.0
    per_lon = per_lat * max(math.cos(math.radians(latitude)), 1e-6)
    return per_lat, per_lon


def _yaw_of(orientation) -> float:
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Station:
    """One buffered sample, in whichever frame its source produced."""

    __slots__ = ('kind', 'a', 'b', 'c', 'yaw', 'fix', 'sigma', 'x', 'y')

    def __init__(self, kind, a, b, c, yaw, fix, sigma, x, y):
        self.kind = kind      # 'map' | 'geodetic'
        self.a = a            # map x   | latitude
        self.b = b            # map y   | longitude
        self.c = c            # map z   | altitude
        self.yaw = yaw
        self.fix = fix
        self.sigma = sigma
        self.x = x            # local metric coordinates, trigger + closure
        self.y = y


class RouteRecorder(Node):
    """Buffer stations as they are driven, then write them out geodetic."""

    def __init__(self) -> None:
        super().__init__('route_recorder')

        self.declare_parameter('source', route_file.SOURCE_ODOMETRY)
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('fix_topic', '/gnss/fix_gated')
        self.declare_parameter('output_path', 'route.yaml')
        self.declare_parameter('sample_dist_m', 1.0)
        self.declare_parameter('sample_yaw_deg', 5.0)
        self.declare_parameter('loop', True)
        # A loop is only closed if the pass came back to where it started.
        self.declare_parameter('loop_closure_tolerance_m', 3.0)
        self.declare_parameter('lane_half_width_m', 2.0)
        self.declare_parameter('shoulder_left_m', 1.0)
        self.declare_parameter('shoulder_right_m', 1.0)
        # Samples below this class are kept but FLAGGED -- rejecting them at
        # record time would throw away the only evidence that a stretch of
        # route is GNSS-marginal (issue #8, review question 5).
        self.declare_parameter('warn_below_fix', 'fixed')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('antenna_frame', 'gnss_link')
        self.declare_parameter('toll_service', '/toLL')
        self.declare_parameter('toll_timeout_s', 5.0)

        self._source = self.get_parameter('source').value
        if self._source not in (route_file.SOURCE_ODOMETRY,
                                route_file.SOURCE_FIX_LEVER_ARM,
                                route_file.SOURCE_RAW_ANTENNA):
            raise ValueError('unknown source %r' % self._source)

        self._sample_dist = float(self.get_parameter('sample_dist_m').value)
        self._sample_yaw = math.radians(
            float(self.get_parameter('sample_yaw_deg').value))
        self._output_path = self.get_parameter('output_path').value
        self._warn_below = self.get_parameter('warn_below_fix').value

        self._stations = []
        self._yaw = None
        self._fix_class = 'none'
        self._sigma = 0.0
        self._lever_arm = None
        self._anchor = None
        self._degraded = 0

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, qos)
        self.create_subscription(
            NavSatFix, self.get_parameter('fix_topic').value,
            self._on_fix, qos)

        # Service calls are made from inside another callback. The client must
        # sit in a DIFFERENT callback group from the service that triggers it,
        # or the executor is still inside the service handler when the toLL
        # response arrives and nothing ever picks it up.
        self._srv_group = MutuallyExclusiveCallbackGroup()
        self._client_group = MutuallyExclusiveCallbackGroup()
        self._toll = self.create_client(
            ToLL, self.get_parameter('toll_service').value,
            callback_group=self._client_group)
        self.create_service(Trigger, '~/save', self._on_save,
                            callback_group=self._srv_group)
        self.create_service(Trigger, '~/discard', self._on_discard,
                            callback_group=self._srv_group)

        if self._source == route_file.SOURCE_FIX_LEVER_ARM:
            self._start_tf()

        self.get_logger().info(
            'route_recorder up: source=%s, trigger every %.2f m or %.1f deg, '
            'writing %s'
            % (self._source, self._sample_dist,
               math.degrees(self._sample_yaw), self._output_path))
        if self._source == route_file.SOURCE_RAW_ANTENNA:
            self.get_logger().warn(
                'source=raw_antenna records the ANTENNA phase centre, not '
                'base_link. Differential-test control only -- the follower '
                'refuses this file.')

    # -- lever arm ---------------------------------------------------------

    def _start_tf(self) -> None:
        """Look the antenna offset up from TF, once, and cache it."""
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_timer = self.create_timer(0.5, self._try_lever_arm)

    def _try_lever_arm(self) -> None:
        base = self.get_parameter('base_frame').value
        antenna = self.get_parameter('antenna_frame').value
        try:
            tf = self._tf_buffer.lookup_transform(
                base, antenna, rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        self._lever_arm = (t.x, t.y)
        self._tf_timer.cancel()
        self.get_logger().info(
            'antenna lever arm %s in %s: (%.3f, %.3f) m -- subtracting it '
            'from every fix' % (antenna, base, t.x, t.y))

    # -- inputs ------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        # Worse axis, not just east: north was 1.7x worse than east in a
        # measured soak here, so cov[0] alone understates what gets written
        # into the route file as sigma_h.
        self._sigma = route_file.horizontal_sigma(msg)
        self._fix_class = route_file.classify_fix(
            msg.status.status, self._sigma)

        if self._source == route_file.SOURCE_ODOMETRY:
            return
        if self._yaw is None:
            return  # no heading yet, so the lever arm cannot be rotated

        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
        if self._source == route_file.SOURCE_FIX_LEVER_ARM:
            if self._lever_arm is None:
                return
            lat, lon = self._subtract_lever_arm(lat, lon, self._yaw)

        x, y = self._local_metric(lat, lon)
        self._consider(Station('geodetic', lat, lon, alt, self._yaw,
                               self._fix_class, self._sigma, x, y))

    def _on_odom(self, msg: Odometry) -> None:
        self._yaw = _yaw_of(msg.pose.pose.orientation)
        if self._source != route_file.SOURCE_ODOMETRY:
            return
        p = msg.pose.pose.position
        self._consider(Station('map', p.x, p.y, p.z, self._yaw,
                               self._fix_class, self._sigma, p.x, p.y))

    def _subtract_lever_arm(self, lat: float, lon: float, yaw: float):
        """Antenna lat/lon -> base_link lat/lon.

        The antenna sits at ``base_link + R(yaw) * t`` with t the TF offset, so
        going the other way is a subtraction of the rotated offset. Done in
        degrees at the fix's own latitude; over a half-metre lever arm the
        spherical approximation is a rounding error.
        """
        tx, ty = self._lever_arm
        east = tx * math.cos(yaw) - ty * math.sin(yaw)
        north = tx * math.sin(yaw) + ty * math.cos(yaw)
        per_lat, per_lon = _metres_per_degree(lat)
        return lat - north / per_lat, lon - east / per_lon

    def _local_metric(self, lat: float, lon: float):
        """Flat-earth metres about the first sample -- trigger + closure only."""
        if self._anchor is None:
            self._anchor = (lat, lon)
        alat, alon = self._anchor
        per_lat, per_lon = _metres_per_degree(alat)
        return (lon - alon) * per_lon, (lat - alat) * per_lat

    # -- sampling ----------------------------------------------------------

    def _consider(self, station: Station) -> None:
        # Never record a station with no usable fix. Originally this only
        # skipped stations BEFORE the first fix; it now skips them anywhere,
        # because reading the raw driver topic means NO_FIX messages reach us
        # instead of being dropped by confidence_gate, and a station recorded
        # from one carries a meaningless position. A gap in the route is
        # recoverable; a station in the wrong place is not.
        if station.fix == 'none':
            return

        if self._stations:
            previous = self._stations[-1]
            moved = math.hypot(station.x - previous.x, station.y - previous.y)
            turned = abs(_angle_diff(station.yaw, previous.yaw))
            if moved < self._sample_dist and turned < self._sample_yaw:
                return

        if self._below_threshold(station.fix):
            self._degraded += 1

        self._stations.append(station)
        if len(self._stations) % 25 == 0:
            self.get_logger().info('%d stations recorded'
                                   % len(self._stations))

    def _below_threshold(self, fix_class: str) -> bool:
        classes = route_file.FIX_CLASSES
        try:
            return classes.index(fix_class) < classes.index(self._warn_below)
        except ValueError:
            return True

    # -- output ------------------------------------------------------------

    def _on_discard(self, _request, response):
        count = len(self._stations)
        self._stations = []
        self._degraded = 0
        self._anchor = None
        response.success = True
        response.message = 'discarded %d stations' % count
        self.get_logger().warn(response.message)
        return response

    def _on_save(self, _request, response):
        try:
            path = self.save()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error('save failed: %s' % exc)
            return response
        response.success = True
        response.message = 'wrote %d samples to %s' % (
            len(self._stations), path)
        self.get_logger().info(response.message)
        return response

    def save(self) -> str:
        """Convert the buffer to geodetic and write the route file."""
        if len(self._stations) < 4:
            raise RuntimeError(
                'only %d stations buffered; a route needs at least 4'
                % len(self._stations))

        shoulder_l = float(self.get_parameter('shoulder_left_m').value)
        shoulder_r = float(self.get_parameter('shoulder_right_m').value)
        datum, geodetic = self._to_geodetic()

        samples = [
            Sample(lat=lat, lon=lon, alt=alt, yaw=station.yaw,
                   fix=station.fix, sigma_h=station.sigma,
                   shoulder_left_m=shoulder_l, shoulder_right_m=shoulder_r)
            for station, (lat, lon, alt) in zip(self._stations, geodetic)
        ]

        route = Route(
            datum=datum,
            loop=self._resolve_loop(),
            lane_half_width_m=float(
                self.get_parameter('lane_half_width_m').value),
            source=self._source,
            frame=(self.get_parameter('base_frame').value
                   if self._source != route_file.SOURCE_RAW_ANTENNA
                   else self.get_parameter('antenna_frame').value),
            recorded=route_file.now_iso(),
            samples=samples,
        )

        path = os.path.abspath(self._output_path)
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        route_file.save(route, path)

        if self._degraded:
            self.get_logger().warn(
                '%d of %d samples were below the %r fix threshold and are '
                'flagged in the file'
                % (self._degraded, len(samples), self._warn_below))
        return path

    def _to_geodetic(self):
        """(datum, [(lat, lon, alt), ...]) for the buffered stations."""
        if self._stations[0].kind == 'geodetic':
            # Fix-driven sources are already geodetic. The datum is only
            # metadata here, so report the origin navsat_transform is using if
            # it is up, and fall back to the first sample if it is not.
            samples = [(s.a, s.b, s.c) for s in self._stations]
            datum = self._datum_or(samples[0])
            return datum, samples

        timeout = float(self.get_parameter('toll_timeout_s').value)
        if not self._toll.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                'toLL service %r is not available -- navsat_transform must be '
                'running to convert a map-frame route to geodetic'
                % self._toll.srv_name)
        datum = self._to_ll(0.0, 0.0, 0.0, timeout)
        return datum, [self._to_ll(s.a, s.b, s.c, timeout)
                       for s in self._stations]

    def _datum_or(self, fallback):
        timeout = float(self.get_parameter('toll_timeout_s').value)
        if not self._toll.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'toLL unavailable; recording the first sample as the datum')
            return fallback
        return self._to_ll(0.0, 0.0, 0.0, timeout)

    def _resolve_loop(self) -> bool:
        """`loop: true` is a claim; check it before writing it down."""
        if not bool(self.get_parameter('loop').value):
            return False
        tolerance = float(
            self.get_parameter('loop_closure_tolerance_m').value)
        first, last = self._stations[0], self._stations[-1]
        gap = math.hypot(last.x - first.x, last.y - first.y)
        if gap > tolerance:
            self.get_logger().warn(
                'loop:=true but the pass ended %.2f m from where it started '
                '(tolerance %.2f m) -- writing loop: false'
                % (gap, tolerance))
            return False
        self.get_logger().info('loop closes to %.2f m' % gap)
        return True

    def _to_ll(self, x: float, y: float, z: float, timeout: float):
        request = ToLL.Request()
        request.map_point = Point(x=float(x), y=float(y), z=float(z))
        future = self._toll.call_async(request)
        # Do NOT spin here: this runs inside a service callback of an executor
        # that is already spinning. Wait for the other callback group to
        # deliver instead.
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError('toLL call timed out after %.1f s'
                                   % timeout)
            time.sleep(0.002)
        ll = future.result().ll_point
        return (ll.latitude, ll.longitude, ll.altitude)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteRecorder()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
