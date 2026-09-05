# Copyright 2026 Outdoor Patrol Team
# Licensed under the Apache License, Version 2.0.
"""Synthetic stack for rehearsing the field validation indoors.

Publishes the same topics, types and frames the real drivers do -- the
:mod:`outdoor_patrol_sim` design rule -- and walks them through a scripted
alley run. The dashboard cannot tell the difference, which is the point.

This exists because the field procedure is expensive to get wrong. A trip to
the alley is three hours, the phases are sequential, and several gates only
fire on failures you cannot conjure on demand outdoors: you cannot ask a real
receiver to report a 180 deg heading error, and you should not find out that
the obstacle gate is misconfigured while a robot is driving at a wall.

So rehearse first::

    ros2 launch outdoor_patrol_validation rehearsal.launch.py scenario:=obstacle

and step the panel through the phases against a stack whose right answer you
already know. Every scenario below is a failure the plan names.

Scenarios
---------

``nominal``
    RTK fixed throughout, heading agrees with travel, clean 30 m run.
    Phases 0-5 should all go green.
``heading_flip``
    ANT1/ANT2 swapped: heading reads 180 deg out. Phase 2 must FAIL and say
    so.
``bad_rtk``
    Sigma wanders over the 5 cm gate. Phase 1 must never reach its hold.
``obstacle``
    A 2.4 m barrier against the left wall at 18 m. Phase 6's retreat/resume
    sequence.
``wrong_side``
    The same barrier, but the follower retreats LEFT. Phase 6 must FAIL on
    ``d_cmd never positive``.
``lever_arm_flipped``
    ``/fromLL`` reports the antenna on the wrong side. Phase 4 must FAIL on
    the sign check.
``gnss_fault``
    Sky view blocked mid-run: sigma climbs, the robot slows, then stops.
    Phase 7.
``driveway``
    The 18 x 18 ft driveway shakedown: a 3.5 m square with 1 m corner
    fillets, driven as a closed circuit. Exercises all four cardinal
    headings and the loop-closure gate, which the straight scenarios cannot.
    Pair it with ``field_dashboard_driveway.yaml``.

Nothing here is used in the field. It is a bench tool, and it publishes on the
real topic names, so **do not run it while the robot is up** -- it would fight
the real drivers for ``/odometry/global``.
"""

from __future__ import annotations

import json
import math
from typing import Optional

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robot_localization.srv import FromLL
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from std_msgs.msg import String
import tf2_ros

#: Alley geometry, matching doc/eng/plans/field-validation-alley.md.
ALLEY_HALF_WIDTH = 2.0
ALLEY_LENGTH = 30.0
OBSTACLE_AT_M = 18.0
OBSTACLE_WIDTH_M = 2.4
#: chassis.yaml gnss_link, the offset Phase 4 has to recover.
ANTENNA_X = 0.28
ANTENNA_Y = -0.42

#: Driveway shakedown: an 18 x 18 ft slab, route a 3.5 m square with 1 m
#: fillets. Matches route_driveway.yaml.
DRIVEWAY_HALF = 1.75
DRIVEWAY_CORNER_R = 1.0

SCENARIOS = ('nominal', 'heading_flip', 'bad_rtk', 'obstacle', 'wrong_side',
             'lever_arm_flipped', 'gnss_fault', 'driveway')


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))


def nmea_checksum(body: str) -> str:
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f'{checksum:02X}'


class FieldRehearsal(Node):
    """A robot that only exists on the topic bus."""

    def __init__(self) -> None:
        super().__init__('field_rehearsal')

        self.declare_parameter('scenario', 'nominal')
        self.declare_parameter('speed_ms', 0.4)
        self.declare_parameter('start_delay_s', 12.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('sigma_m', 0.021)
        self.declare_parameter('datum_lat', 47.5431522)
        self.declare_parameter('datum_lon', -121.8803670)
        #: Off when robot_state_publisher is running: it publishes the same
        #: frames from the real URDF, and two sources for one static
        #: transform is a TF_REPEATED_DATA warning storm.
        self.declare_parameter('publish_static_tf', True)

        self._scenario = str(self.get_parameter('scenario').value)
        if self._scenario not in SCENARIOS:
            self.get_logger().warning(
                f'unknown scenario {self._scenario!r}, using nominal '
                f'(choose from {", ".join(SCENARIOS)})')
            self._scenario = 'nominal'

        self._speed = float(self.get_parameter('speed_ms').value)
        self._start_delay = float(self.get_parameter('start_delay_s').value)
        self._base_sigma = float(self.get_parameter('sigma_m').value)
        self._lat0 = float(self.get_parameter('datum_lat').value)
        self._lon0 = float(self.get_parameter('datum_lon').value)

        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self._fix_pub = self.create_publisher(
            NavSatFix, '/um982_driver/fix', reliable)
        self._gated_pub = self.create_publisher(
            NavSatFix, '/gnss/fix_gated', reliable)
        self._nmea_pub = self.create_publisher(
            Sentence, '/um982_driver/nmea_sentence', qos_profile_sensor_data)
        self._heading_pub = self.create_publisher(
            Imu, '/gnss/heading', qos_profile_sensor_data)
        self._odom_pub = self.create_publisher(
            Odometry, '/odometry/global', reliable)
        self._status_pub = self.create_publisher(
            String, '/route_follower/status', reliable)
        self._scan_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', reliable)
        self._tf = tf2_ros.TransformBroadcaster(self)
        self._static_tf = tf2_ros.StaticTransformBroadcaster(self)
        if bool(self.get_parameter('publish_static_tf').value):
            self._publish_static_frames()

        self._fromll = self.create_service(FromLL, '/fromLL', self._on_fromll)

        self._t = 0.0
        self._s = 0.0
        self._d_cmd = 0.0
        self._state = 'driving'
        self._resume_from: Optional[float] = None
        self._fault_start: Optional[float] = None

        period = 1.0 / float(self.get_parameter('rate_hz').value)
        self._dt = period
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'rehearsing scenario {self._scenario!r}: '
            f'{self._start_delay:.0f} s stationary, then '
            f'{self._route_length():.1f} m at {self._speed:.2f} m/s')

    # -- scripted world ----------------------------------------------------

    def _route_length(self) -> float:
        if self._scenario == 'driveway':
            side = DRIVEWAY_HALF - DRIVEWAY_CORNER_R
            return 8.0 * side + 2.0 * math.pi * DRIVEWAY_CORNER_R
        return ALLEY_LENGTH

    def _pose_at(self, s: float, lateral: float):
        """(x, y, yaw) at arc length `s`, displaced `lateral` to the left.

        The straight scenarios run along +x, so the alley case is trivial.
        The driveway walks a rounded square clockwise, which is what makes it
        worth rehearsing: it visits all four cardinal headings, so a heading
        error shows up as a rotated square rather than hiding in a single
        direction the way it can on a straight run.
        """
        if self._scenario != 'driveway':
            return s, lateral, 0.0

        side = DRIVEWAY_HALF - DRIVEWAY_CORNER_R
        r = DRIVEWAY_CORNER_R
        straight = 2.0 * side
        arc = r * math.pi / 2.0
        leg = straight + arc
        s = s % (4.0 * leg)
        index = int(s // leg)
        within = s - index * leg
        # Clockwise from the top-right corner, heading -y first.
        base = -index * math.pi / 2.0
        if within < straight:
            yaw = base - math.pi / 2.0
            along = within - side
            cx, cy = DRIVEWAY_HALF, -along
        else:
            a = (within - straight) / r
            yaw = base - math.pi / 2.0 - a
            centre = (side, -side)
            angle = -a
            cx = centre[0] + r * math.cos(angle)
            cy = centre[1] + r * math.sin(angle)
        # Rotate the canonical first leg into place for this side.
        c, sn = math.cos(base), math.sin(base)
        x = c * cx - sn * cy
        y = sn * cx + c * cy
        # Left normal of the heading.
        x += lateral * -math.sin(yaw)
        y += lateral * math.cos(yaw)
        return x, y, yaw

    def _publish_static_frames(self) -> None:
        """base_link -> lidar_link and gnss_link, from chassis.yaml.

        Without lidar_link, RViz has no frame to draw ``/scan`` in and the
        alley walls simply never appear -- which looks like a broken lidar
        rather than a missing transform.
        """
        transforms = []
        for child, (x, y, z, yaw) in {
            'lidar_link': (0.512, 0.0, 0.05, math.pi),
            'gnss_link': (ANTENNA_X, ANTENNA_Y, 0.18, 0.0),
        }.items():
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = child
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation = quaternion_from_yaw(yaw)
            transforms.append(transform)
        self._static_tf.sendTransform(transforms)

    @property
    def _driving(self) -> bool:
        return self._t >= self._start_delay and self._s < self._route_length()

    def _sigma(self) -> float:
        """Horizontal 1-sigma, per scenario."""
        if self._scenario == 'bad_rtk':
            # Wanders across the 5 cm gate every ~20 s, so the soak hold keeps
            # restarting and never completes.
            return 0.02 + 0.05 * (1.0 + math.sin(self._t * 0.3)) / 2.0
        if self._scenario == 'gnss_fault':
            if self._fault_start is None and self._s > 12.0:
                self._fault_start = self._t
            if self._fault_start is not None:
                # Degrades with TIME, not distance. A distance-driven ramp
                # freezes the moment the robot stops, so sigma would stall
                # just short of sigma_stop and the fault would never complete.
                return min(0.35, 0.02 + 0.02 * (self._t - self._fault_start))
        return self._base_sigma

    def _gga_quality(self) -> int:
        sigma = self._sigma()
        if sigma > 0.15:
            return 1
        if sigma > 0.05:
            return 5
        return 4

    def _reported_yaw(self) -> float:
        """What the receiver claims, in ENU radians.

        The straight scenarios run along +x, so the honest answer is 0 (East).
        The driveway follows the square, which is the point of it: a heading
        error that a single straight leg could hide shows up on at least one
        of the four sides.
        """
        _, _, yaw = self._pose_at(self._s, 0.0)
        if self._scenario == 'heading_flip':
            return yaw + math.pi
        return yaw

    def _speed_now(self) -> float:
        if not self._driving:
            return 0.0
        if self._scenario == 'gnss_fault':
            sigma = self._sigma()
            if sigma >= 0.15:
                return 0.0
            if sigma >= 0.05:
                # The follower's linear ramp between sigma_slow and sigma_stop.
                return self._speed * (1.0 - (sigma - 0.05) / 0.10)
        return self._speed

    def _update_offset(self) -> None:
        """Drive the retreat/resume sequence for the obstacle scenarios."""
        if self._scenario not in ('obstacle', 'wrong_side'):
            self._state = 'driving' if self._driving else (
                'finished' if self._s >= ALLEY_LENGTH else 'driving')
            return

        sign = 1.0 if self._scenario == 'wrong_side' else -1.0
        trigger = OBSTACLE_AT_M - 8.0
        clear = OBSTACLE_AT_M + 2.0

        if self._s < trigger:
            self._state = 'driving'
            self._d_cmd = 0.0
        elif self._s < OBSTACLE_AT_M:
            self._state = 'retreating'
            ramp = min(1.0, (self._s - trigger) / 2.0)
            self._d_cmd = sign * 1.2 * ramp
        elif self._s < clear:
            self._state = 'retreating'
            self._d_cmd = sign * 1.2
        else:
            if self._resume_from is None:
                self._resume_from = self._s
            travelled = self._s - self._resume_from
            if travelled < 2.0:
                self._state = 'resuming'
                self._d_cmd = sign * 1.2 * (1.0 - travelled / 2.0)
            else:
                self._state = 'driving'
                self._d_cmd = 0.0

    def _scan(self) -> LaserScan:
        """360 beams: two parallel walls, plus the barrier when there is one.

        Nose-forward, matching what `/scan` looks like once ``lidar_link``'s
        yaw-180 mount has been applied.
        """
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.06
        scan.range_max = 25.0

        y = self._d_cmd
        left_gap = ALLEY_HALF_WIDTH - y
        right_gap = ALLEY_HALF_WIDTH + y
        # Distance to the barrier, which spans the left 2.4 m of the alley.
        obstacle_edge = ALLEY_HALF_WIDTH - OBSTACLE_WIDTH_M
        blocked_ahead = (self._scenario in ('obstacle', 'wrong_side')
                         and y > obstacle_edge - 0.3
                         and self._s < OBSTACLE_AT_M)

        ranges = []
        for i in range(360):
            # The C1 is mounted yaw-180, so scan angle 0 points BACKWARD at
            # the robot body. Reproduce that here rather than publishing a
            # convenient nose-forward scan: it is the convention every
            # consumer has to cope with, so the rehearsal has to test it.
            angle = scan.angle_min + i * scan.angle_increment
            body = angle + math.pi
            sin_a, cos_a = math.sin(body), math.cos(body)
            candidates = []
            if sin_a > 1e-3:
                candidates.append(left_gap / sin_a)
            if sin_a < -1e-3:
                candidates.append(right_gap / -sin_a)
            if cos_a > 1e-3:
                ahead = (OBSTACLE_AT_M - self._s) / cos_a if blocked_ahead \
                    else (ALLEY_LENGTH + 5.0 - self._s) / cos_a
                candidates.append(max(0.2, ahead))
            ranges.append(float(min(candidates)) if candidates
                          else float('inf'))
        scan.ranges = ranges
        return scan

    # -- publish -----------------------------------------------------------

    def _tick(self) -> None:
        speed = self._speed_now()
        self._t += self._dt
        self._s += speed * self._dt
        self._update_offset()
        if self._s >= self._route_length():
            self._state = 'finished'

        now = self.get_clock().now().to_msg()
        sigma = self._sigma()
        variance = sigma * sigma

        x, y, yaw = self._pose_at(self._s, self._d_cmd)
        # The antenna sits at base_link + R(yaw) * lever arm.
        ax = x + ANTENNA_X * math.cos(yaw) - ANTENNA_Y * math.sin(yaw)
        ay = y + ANTENNA_X * math.sin(yaw) + ANTENNA_Y * math.cos(yaw)
        # Metres to degrees at the datum. Good enough: the dashboard only ever
        # differences these, it never navigates on them.
        per_lat = 111132.0
        per_lon = 111320.0 * math.cos(math.radians(self._lat0))
        lat = self._lat0 + ay / per_lat
        lon = self._lon0 + ax / per_lon

        fix = NavSatFix()
        fix.header.stamp = now
        fix.header.frame_id = 'gnss_link'
        fix.latitude, fix.longitude, fix.altitude = lat, lon, 250.0
        fix.position_covariance = [
            variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance * 4.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(fix)

        gated = NavSatFix()
        gated.header = fix.header
        gated.latitude, gated.longitude, gated.altitude = lat, lon, 250.0
        # confidence_gate's x1000 inflation above the 5 cm threshold.
        inflated = variance * (1000.0 if sigma > 0.05 else 1.0)
        gated.position_covariance = [
            inflated, 0.0, 0.0, 0.0, inflated, 0.0, 0.0, 0.0, inflated * 4.0]
        gated.position_covariance_type = fix.position_covariance_type
        self._gated_pub.publish(gated)

        quality = self._gga_quality()
        body = (f'GNGGA,{self._t:09.2f},4732.58913274,N,12152.82201833,W,'
                f'{quality},30,0.5,250.6193,M,-21.0746,M,1.4,4053')
        sentence = Sentence()
        sentence.header.stamp = now
        sentence.header.frame_id = 'gnss_link'
        sentence.sentence = f'${body}*{nmea_checksum(body)}'
        self._nmea_pub.publish(sentence)

        heading = Imu()
        heading.header.stamp = now
        heading.header.frame_id = 'base_link'
        heading.orientation = quaternion_from_yaw(self._reported_yaw())
        heading.orientation_covariance[0] = -1.0
        heading.angular_velocity_covariance[0] = -1.0
        heading.linear_acceleration_covariance[0] = -1.0
        self._heading_pub.publish(heading)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = quaternion_from_yaw(yaw)
        odom.twist.twist.linear.x = speed
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = quaternion_from_yaw(yaw)
        self._tf.sendTransform(transform)

        self._scan_pub.publish(self._scan())

        command = Twist()
        command.linear.x = speed
        self._cmd_pub.publish(command)

        # Only publish follower status once the run is under way -- before
        # that there is no follower, and the dashboard should show `idle`.
        if self._t >= self._start_delay:
            self._status_pub.publish(String(data=json.dumps({
                'state': self._state,
                's': round(self._s, 3),
                'travelled': round(self._s, 3),
                'odom_distance': round(self._s, 3),
                'lateral': round(self._d_cmd, 4),
                'cross_track': 0.03,
                'd_cmd': round(self._d_cmd, 4),
                'd_target': round(self._d_cmd, 3),
                'blocked': [],
                'speed': round(speed, 3),
                'omega': 0.0,
                'sigma_h': round(sigma, 4),
            })))

    def _on_fromll(self, request, response):
        """Stand in for navsat_transform, so Phase 4 has something to measure.

        Returns where the antenna really is, so the dashboard should recover
        exactly the ``chassis.yaml`` lever arm -- or, in the
        ``lever_arm_flipped`` scenario, its mirror image, which the sign check
        has to catch.
        """
        sign = -1.0 if self._scenario == 'lever_arm_flipped' else 1.0
        response.map_point.x = self._s + ANTENNA_X
        response.map_point.y = self._d_cmd + sign * ANTENNA_Y
        response.map_point.z = 0.0
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldRehearsal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
