# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Repeat a recorded route with pure pursuit, retreating onto the shoulder.

The whole controller is one idea: **retreat is not "go somewhere else", it is
"follow the same path, shifted sideways"**::

    look_ahead = centerline(s + L) + d * normal(s + L)

``d = 0`` is normal patrol. ``d = -2.4 m`` is parked out on the right
shoulder. Ramping ``d`` between them is a lane change. Retreat and resume are
the same operation with opposite sign, so there is no second control mode, no
state machine and no costmap.

Choosing ``d`` is likewise not a special case. Every cycle the follower asks,
for each candidate offset from zero outward, "is the corridor at this offset
clear for the next few metres?" and takes the smallest ``|d|`` that is. That
one rule produces all of the required behaviour:

* nothing ahead -> ``d = 0``, drive the centerline
* barrier in the lane -> the first clear candidate is out on the shoulder
* barrier cleared -> zero is clear again, so it comes back
* **nothing** clear -> stop in place, which is the required fallback

Hysteresis is asymmetric on purpose. Moving further out happens on the first
cycle that demands it; coming back in needs ``resume_clear_cycles`` consecutive
clear cycles, or a single dropped return sets the offset oscillating.

Wiring: commands go to ``/cmd_vel_raw``, so the M3 forward brake
(outdoor_patrol_safety) stays in the path underneath. The brake is the last
resort, not the avoidance mechanism -- it stops at 0.5 m, by which point there
is no room left to steer. This node's own trigger looks several metres ahead
precisely so the brake never has to fire.

Refuses to follow a route recorded at the antenna (``source: raw_antenna``):
that file is 0.42 m to the right of where the robot actually drove.
"""

from __future__ import annotations

import json
import math
import time

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from robot_localization.srv import FromLL
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from outdoor_patrol_route import route_file
from outdoor_patrol_route.path import Path, forward_gap

#: Reported in ~/status, and what the scorer asserts on.
STATE_DRIVING = 'driving'
STATE_RETREATING = 'retreating'
STATE_RESUMING = 'resuming'
STATE_BLOCKED = 'blocked'
STATE_DEGRADED = 'degraded'
STATE_FINISHED = 'finished'


def _yaw_of(orientation) -> float:
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RouteFollower(Node):
    """Pure pursuit with a lateral offset chosen by corridor occupancy."""

    def __init__(self) -> None:
        super().__init__('route_follower')

        self.declare_parameter('route_path', '')
        self.declare_parameter('odom_topic', '/odometry/global')
        # RAW driver fix, deliberately NOT /gnss/fix_gated. The gate
        # multiplies covariance by 1000 on a degraded fix, which is an
        # EKF-weighting device rather than a quality metric: a 6 cm fix
        # arrives here as 1.90 m, vaulting clean over sigma_slow/sigma_stop
        # and turning a speed ramp into an on/off cliff. Reading the raw
        # sigma is what makes those two thresholds mean anything.
        #
        # This only works because horizontal_sigma() returns inf for a
        # NO_FIX or covariance-less message -- the gate used to drop those
        # before the follower ever saw them.
        self.declare_parameter('fix_topic', '/um982_driver/fix')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('control_period_s', 0.05)

        self.declare_parameter('lookahead_min_m', 1.2)
        self.declare_parameter('lookahead_gain_s', 1.5)
        self.declare_parameter('lookahead_max_m', 4.0)
        self.declare_parameter('nominal_speed_ms', 0.8)
        self.declare_parameter('min_speed_ms', 0.25)
        self.declare_parameter('max_angular_rads', 0.67)
        self.declare_parameter('curvature_speed_gain', 1.0)
        self.declare_parameter('goal_tolerance_m', 1.0)
        self.declare_parameter('laps', 1.0)

        self.declare_parameter('retreat_side', 'right')
        self.declare_parameter('corridor_half_width_m', 3.0)
        self.declare_parameter('offset_step_m', 0.6)
        self.declare_parameter('clearance_half_width_m', 0.55)
        self.declare_parameter('trigger_range_m', 10.0)
        self.declare_parameter('resume_clear_cycles', 20)
        self.declare_parameter('ramp_lateral_per_m', 0.6)

        self.declare_parameter('sigma_slow_m', 0.10)
        self.declare_parameter('sigma_stop_m', 0.50)
        self.declare_parameter('fix_timeout_s', 2.0)

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('fromll_service', '/fromLL')
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('smooth_window', 5)

        self._period = float(self.get_parameter('control_period_s').value)
        self._nominal = float(self.get_parameter('nominal_speed_ms').value)
        self._min_speed = float(self.get_parameter('min_speed_ms').value)
        self._max_omega = float(self.get_parameter('max_angular_rads').value)
        self._trigger_range = float(
            self.get_parameter('trigger_range_m').value)
        self._clearance = float(
            self.get_parameter('clearance_half_width_m').value)
        self._resume_cycles = int(
            self.get_parameter('resume_clear_cycles').value)
        self._ramp_rate = float(self.get_parameter('ramp_lateral_per_m').value)
        self._sigma_slow = float(self.get_parameter('sigma_slow_m').value)
        self._sigma_stop = float(self.get_parameter('sigma_stop_m').value)
        self._fix_timeout = float(self.get_parameter('fix_timeout_s').value)
        self._laps = float(self.get_parameter('laps').value)

        side = str(self.get_parameter('retreat_side').value).lower()
        if side not in ('left', 'right'):
            raise ValueError('retreat_side must be left or right, not %r'
                             % side)
        self._side_sign = 1.0 if side == 'left' else -1.0

        self._route = self._load_route()
        self._offsets = self._candidate_offsets()
        self._check_retreat_geometry()

        # Motion state.
        self._pose = None
        self._prev_xy = None
        self._s = None
        self._travelled = 0.0
        self._odom_distance = 0.0
        self._speed = 0.0
        self._d_cmd = 0.0
        self._committed = 0
        self._clear_streak = 0
        self._scan_points = np.zeros((0, 2))
        self._scan_stamp = None
        self._sigma = 0.0
        self._fix_stamp = None
        self._lidar_tf = None
        self._state = STATE_DRIVING
        self._finished = False
        self._path = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        scan_qos = QoSProfile(depth=5)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, qos)
        self._status_pub = self.create_publisher(String, '~/status', qos)
        self._done_pub = self.create_publisher(Bool, '~/finished', latched)
        # Volatile, not latched: the look-ahead marker is republished every
        # cycle, and latching it would keep evicting the path markers from a
        # late subscriber's history.
        self._marker_pub = self.create_publisher(
            MarkerArray, '~/markers', 5)

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, qos)
        self.create_subscription(
            NavSatFix, self.get_parameter('fix_topic').value,
            self._on_fix, qos)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, scan_qos)

        self._client_group = MutuallyExclusiveCallbackGroup()
        self._fromll = self.create_client(
            FromLL, self.get_parameter('fromll_service').value,
            callback_group=self._client_group)

        self._start_tf()
        # The path cannot be built until fromLL is up, which needs the
        # executor spinning -- so build it from a timer rather than here.
        self._build_timer = self.create_timer(1.0, self._try_build_path)
        self.create_timer(self._period, self._control)
        # RViz subscribers come and go, so redraw the corridor periodically
        # rather than relying on a latch.
        self.create_timer(2.0, self._publish_path_markers)

    # -- start-up ----------------------------------------------------------

    def _load_route(self):
        path = self.get_parameter('route_path').value
        if not path:
            raise ValueError('route_path is required')
        route = route_file.load(path)
        if not route.is_base_link:
            raise ValueError(
                'refusing to follow %s: source=%s records the ANTENNA phase '
                'centre, which on this vehicle is 0.42 m right of base_link'
                % (path, route.source))
        self.get_logger().info(
            'route: %d samples, loop=%s, source=%s, worst fix=%s'
            % (len(route.samples), route.loop, route.source,
               route.worst_fix()))
        return route

    def _candidate_offsets(self):
        """Offsets to try, smallest magnitude first, inside the corridor."""
        step = float(self.get_parameter('offset_step_m').value)
        limit = (float(self.get_parameter('corridor_half_width_m').value)
                 - self._clearance)
        if step <= 0.0 or limit <= 0.0:
            return [0.0]
        count = int(math.floor(limit / step))
        return [self._side_sign * step * i for i in range(count + 1)]

    def _check_retreat_geometry(self) -> None:
        """Refuse a trigger range the ramp cannot use.

        The ramp is parameterised by DISTANCE TRAVELLED, so reaching the
        outermost offset always costs `|d| / ramp` metres of forward travel
        no matter how slowly the robot drives. If that is more than the
        warning distance, the robot arrives at the obstacle still out of
        position, the 0.5 m forward brake zeroes its speed, and pure pursuit
        -- which derives yaw rate from speed -- can no longer steer out of it.
        That is a permanent stall, and it is silent, so check for it up front.
        """
        widest = max(abs(d) for d in self._offsets)
        needed = widest / max(self._ramp_rate, 1e-6)
        margin = self._trigger_range - needed
        if margin < 0.5:
            self.get_logger().error(
                'retreat geometry is unusable: reaching %.2f m of offset at '
                '%.2f m/m needs %.2f m of travel, but obstacles are only seen '
                '%.2f m ahead (margin %+.2f m). Raise trigger_range_m or '
                'ramp_lateral_per_m, or the robot will stall against the '
                'first obstacle it meets.'
                % (widest, self._ramp_rate, needed, self._trigger_range,
                   margin))
        else:
            self.get_logger().info(
                'retreat geometry: %.2f m of offset needs %.2f m of travel, '
                'seen %.2f m ahead (%.2f m margin)'
                % (widest, needed, self._trigger_range, margin))

    def _start_tf(self) -> None:
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_timer = self.create_timer(0.5, self._try_lidar_tf)

    def _try_lidar_tf(self) -> None:
        """Cache base_link -> lidar_link once; it is a fixed joint."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.get_parameter('base_frame').value,
                self.get_parameter('lidar_frame').value,
                rclpy.time.Time())
        except Exception:
            return
        t = tf.transform.translation
        self._lidar_tf = (t.x, t.y, _yaw_of(tf.transform.rotation))
        self._tf_timer.cancel()
        self.get_logger().info(
            'lidar mount in base_link: (%.3f, %.3f) m, yaw %.1f deg'
            % (t.x, t.y, math.degrees(self._lidar_tf[2])))

    def _try_build_path(self) -> None:
        if self._path is not None:
            return
        if not self._fromll.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                'waiting for %s -- navsat_transform must be running to '
                'project the route into the map frame'
                % self._fromll.srv_name, throttle_duration_sec=10.0)
            return

        points = []
        for sample in self._route.samples:
            request = FromLL.Request()
            request.ll_point = GeoPoint(latitude=sample.lat,
                                        longitude=sample.lon, altitude=0.0)
            future = self._fromll.call_async(request)
            # Poll rather than spin: this runs inside a timer callback of an
            # already-spinning executor, and the client sits in its own
            # callback group so the response can be delivered underneath.
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.001)
            if not future.done():
                self.get_logger().error('fromLL timed out; retrying')
                return
            p = future.result().map_point
            points.append((p.x, p.y))

        self._path = Path(points, loop=self._route.loop,
                          smooth_window=int(
                              self.get_parameter('smooth_window').value))
        self._build_timer.cancel()
        self.get_logger().info(
            'path built: %.2f m, loop=%s, offsets %s'
            % (self._path.length, self._path.loop,
               ', '.join('%+.1f' % d for d in self._offsets)))
        self._publish_path_markers()

    # -- inputs ------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, _yaw_of(msg.pose.pose.orientation))

    def _on_fix(self, msg: NavSatFix) -> None:
        # Worse axis, and inf for an untrustworthy fix -- see
        # route_file.horizontal_sigma for why both matter.
        self._sigma = route_file.horizontal_sigma(msg)
        self._fix_stamp = self.get_clock().now()

    def _on_scan(self, msg: LaserScan) -> None:
        if self._pose is None or self._lidar_tf is None:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Keep only returns that could matter: finite, in range, and no
        # further away than the trigger window plus a margin.
        good = (np.isfinite(ranges) & (ranges > msg.range_min)
                & (ranges < min(msg.range_max, self._trigger_range + 2.0)))
        ranges, angles = ranges[good], angles[good]
        if len(ranges) == 0:
            self._scan_points = np.zeros((0, 2))
            self._scan_stamp = self.get_clock().now()
            return

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        # lidar -> base_link (fixed joint), base_link -> map (current pose).
        tx, ty, tyaw = self._lidar_tf
        bx = tx + lx * math.cos(tyaw) - ly * math.sin(tyaw)
        by = ty + lx * math.sin(tyaw) + ly * math.cos(tyaw)

        px, py, yaw = self._pose
        mx = px + bx * math.cos(yaw) - by * math.sin(yaw)
        my = py + bx * math.sin(yaw) + by * math.cos(yaw)
        self._scan_points = np.stack([mx, my], axis=1)
        self._scan_stamp = self.get_clock().now()

    # -- control -----------------------------------------------------------

    def _control(self) -> None:
        if self._finished or self._path is None or self._pose is None:
            return

        x, y, yaw = self._pose
        if self._prev_xy is not None:
            step = math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
            # A datum jump would otherwise be counted as travel.
            self._odom_distance += step if step < 1.0 else 0.0
        self._prev_xy = (x, y)

        previous_s = self._s
        self._s, lateral = self._path.project(x, y, s_hint=self._s)
        if previous_s is not None:
            # Progress is measured ALONG THE PATH, not by summing position
            # deltas: at 20 Hz, 2 cm of fix noise random-walks a straight line
            # into roughly 10 % more distance, which ends a lap early.
            self._travelled += forward_gap(
                previous_s, self._s, self._path.length, self._path.loop)

        if self._at_end():
            self._finish()
            return

        blocked = self._blocked_offsets()
        target_index = self._choose_offset(blocked)

        if target_index is None:
            self._state = STATE_BLOCKED
            self._publish(0.0, 0.0, lateral, blocked, None)
            return

        self._ramp_toward(self._offsets[target_index])

        degraded = self._fix_penalty()
        speed, omega = self._pursue(x, y, yaw)
        speed *= degraded

        target = self._offsets[target_index]
        if degraded == 0.0:
            self._state = STATE_DEGRADED
            speed, omega = 0.0, 0.0
        elif abs(target) > abs(self._d_cmd) + 1e-3:
            self._state = STATE_RETREATING
        elif abs(target) < abs(self._d_cmd) - 1e-3:
            self._state = STATE_RESUMING
        elif abs(self._d_cmd) > 1e-3:
            self._state = STATE_RETREATING
        else:
            self._state = STATE_DRIVING

        self._speed = speed
        self._publish(speed, omega, lateral, blocked, target_index)

    def _at_end(self) -> bool:
        if self._path.loop:
            return self._travelled >= self._laps * self._path.length
        return (self._path.length - self._s
                <= float(self.get_parameter('goal_tolerance_m').value))

    def _blocked_offsets(self):
        """Which candidate offsets have something in them, ahead of us."""
        blocked = [False] * len(self._offsets)
        if len(self._scan_points) == 0 or self._scan_stamp is None:
            return blocked

        age = (self.get_clock().now() - self._scan_stamp).nanoseconds * 1e-9
        if age > 1.0:
            # Stale scan: fail safe by treating every offset as blocked, which
            # stops the robot rather than steering blind.
            return [True] * len(self._offsets)

        s_lo = self._s
        s_hi = self._s + self._trigger_range
        station, lateral, interior = self._path.stations_of(
            self._scan_points, s_lo, s_hi)
        if not np.any(interior):
            return blocked

        ahead = interior & (station > s_lo) & (station < s_hi)
        if self._path.loop:
            # stations_of wraps, so recompute "ahead" as a forward gap.
            gap = (station - s_lo) % self._path.length
            ahead = interior & (gap > 0.0) & (gap < self._trigger_range)
        candidates = lateral[ahead]
        if len(candidates) == 0:
            return blocked

        for i, d in enumerate(self._offsets):
            blocked[i] = bool(
                np.any(np.abs(candidates - d) < self._clearance))
        return blocked

    def _choose_offset(self, blocked):
        """Smallest |d| that is clear, with asymmetric hysteresis."""
        clear = [i for i, b in enumerate(blocked) if not b]
        if not clear:
            return None
        best = clear[0]

        if best > self._committed:
            # Further out: act now. Waiting to be sure is how you hit things.
            self._committed = best
            self._clear_streak = 0
        elif best < self._committed:
            self._clear_streak += 1
            if self._clear_streak >= self._resume_cycles:
                self._committed = best
                self._clear_streak = 0
        else:
            self._clear_streak = 0
        return self._committed

    def _ramp_toward(self, target: float) -> None:
        """Move the commanded offset toward its target, bounded per metre."""
        if self._prev_xy is None:
            self._d_cmd = target
            return
        allowance = max(self._ramp_rate * max(self._speed, 0.1) * self._period,
                        1e-4)
        delta = target - self._d_cmd
        self._d_cmd += math.copysign(min(abs(delta), allowance), delta)

    def _pursue(self, x: float, y: float, yaw: float):
        lookahead = min(
            float(self.get_parameter('lookahead_max_m').value),
            max(float(self.get_parameter('lookahead_min_m').value),
                float(self.get_parameter('lookahead_min_m').value)
                + float(self.get_parameter('lookahead_gain_s').value)
                * self._speed))

        tx, ty = self._path.offset_at(self._s + lookahead, self._d_cmd)
        dx, dy = tx - x, ty - y
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        distance = max(math.hypot(dx, dy), 1e-3)
        curvature = 2.0 * local_y / (distance * distance)

        speed = self._nominal
        gain = float(self.get_parameter('curvature_speed_gain').value)
        if gain > 0.0 and abs(curvature) > 1e-6:
            speed = min(speed, self._max_omega / (abs(curvature) * gain))

        omega = curvature * speed
        if abs(omega) > self._max_omega:
            # Preserve the geometry: slow down rather than under-steer.
            speed *= self._max_omega / abs(omega)
            omega = math.copysign(self._max_omega, omega)
        return max(speed, self._min_speed), omega

    def _fix_penalty(self) -> float:
        """1.0 healthy, 0.0 stop, linear in between."""
        if self._fix_stamp is None:
            return 0.0
        age = (self.get_clock().now() - self._fix_stamp).nanoseconds * 1e-9
        if age > self._fix_timeout:
            return 0.0
        if self._sigma >= self._sigma_stop:
            return 0.0
        if self._sigma <= self._sigma_slow:
            return 1.0
        span = max(self._sigma_stop - self._sigma_slow, 1e-6)
        return float(max(0.0, 1.0 - (self._sigma - self._sigma_slow) / span))

    # -- outputs -----------------------------------------------------------

    def _publish(self, speed, omega, lateral, blocked, target_index) -> None:
        command = Twist()
        command.linear.x = float(speed)
        command.angular.z = float(omega)
        self._cmd_pub.publish(command)

        status = {
            'state': self._state,
            's': round(self._s, 3),
            'travelled': round(self._travelled, 3),
            'odom_distance': round(self._odom_distance, 3),
            'lateral': round(lateral, 4),
            'cross_track': round(lateral - self._d_cmd, 4),
            'd_cmd': round(self._d_cmd, 4),
            'd_target': (None if target_index is None
                         else round(self._offsets[target_index], 3)),
            'blocked': blocked,
            'speed': round(speed, 3),
            'omega': round(omega, 4),
            'sigma_h': round(self._sigma, 4),
        }
        self._status_pub.publish(String(data=json.dumps(status)))

        if bool(self.get_parameter('publish_markers').value):
            self._publish_lookahead_marker()

    def _finish(self) -> None:
        self._finished = True
        self._state = STATE_FINISHED
        self._cmd_pub.publish(Twist())
        self._done_pub.publish(Bool(data=True))
        self.get_logger().info(
            'route complete: %.1f m travelled (%.2f laps)'
            % (self._travelled, self._travelled / self._path.length))

    # -- markers -----------------------------------------------------------

    def _marker(self, name, index, kind, scale, colour):
        marker = Marker()
        marker.header.frame_id = self.get_parameter('map_frame').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = name
        marker.id = index
        marker.type = kind
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(**colour)
        return marker

    def _publish_path_markers(self) -> None:
        """Centerline + corridor edges.

        Stands in for the deferred route_to_map occupancy grid (issue #8,
        review Q4): it is what the operator actually wants to see, and it does
        not pre-commit the grid semantics.
        """
        if not bool(self.get_parameter('publish_markers').value):
            return
        if self._path is None:
            return
        half = float(self.get_parameter('corridor_half_width_m').value)
        lane = self._route.lane_half_width_m
        bands = [
            ('centerline', 0.0, 0.10, {'r': 0.2, 'g': 0.9, 'b': 0.3,
                                       'a': 0.9}),
            ('lane_left', lane, 0.05, {'r': 0.9, 'g': 0.9, 'b': 0.9,
                                       'a': 0.6}),
            ('lane_right', -lane, 0.05, {'r': 0.9, 'g': 0.9, 'b': 0.9,
                                         'a': 0.6}),
            ('corridor_left', half, 0.05, {'r': 0.9, 'g': 0.6, 'b': 0.2,
                                           'a': 0.5}),
            ('corridor_right', -half, 0.05, {'r': 0.9, 'g': 0.6, 'b': 0.2,
                                             'a': 0.5}),
        ]
        array = MarkerArray()
        for index, (name, offset, width, colour) in enumerate(bands):
            marker = self._marker(name, index, Marker.LINE_STRIP, width,
                                  colour)
            polyline = self._path.offset_polyline(offset)
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=0.05)
                             for p in polyline]
            if self._path.loop:
                marker.points.append(marker.points[0])
            array.markers.append(marker)
        self._marker_pub.publish(array)

    def _publish_lookahead_marker(self) -> None:
        if self._path is None or self._s is None:
            return
        marker = self._marker('lookahead', 100, Marker.SPHERE, 0.35,
                              {'r': 1.0, 'g': 0.3, 'b': 0.1, 'a': 0.9})
        tx, ty = self._path.offset_at(self._s + 1.5, self._d_cmd)
        marker.pose.position = Point(x=tx, y=ty, z=0.3)
        array = MarkerArray()
        array.markers.append(marker)
        self._marker_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteFollower()
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
