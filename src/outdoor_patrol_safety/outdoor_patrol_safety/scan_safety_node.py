# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Geometric forward-obstacle safety brake for a 2D LiDAR (M3, ADR-013).

Subscribes a raw command (``cmd_vel_in``) and the LiDAR ``scan``, and
republishes a gated command (``cmd_vel_out``) that zeros *forward* velocity
when a return falls inside a forward angular sector closer than a stop
distance. Reverse and rotation pass through unchanged so the operator or
planner can always back out of a stop. Fails safe: a missing or stale scan
blocks forward motion. Deliberately dumb and geometric (no costmap) -- the M3
safety layer that stays in front of Nav2 in later milestones.

The gate is re-evaluated on a TIMER, not on command arrival. Gating only on
arrival cannot stop a robot that is ALREADY moving: a latching command source
(``teleop_twist_keyboard`` publishes exactly one Twist per keypress) sends a
single clear-path command, the chassis holds it, and no later message ever
arrives to gate against the obstacle that appears afterwards. So the last raw
command is HELD and re-gated against the freshest scan at
``control_period_s``.

A held command expires after ``cmd_timeout_s``, which mirrors the firmware's
own ``CMD_VEL_TIMEOUT_MS``: on expiry the node emits a single zero Twist and
then goes silent, so a dead or paused command source stops the robot here and
the firmware watchdog still fails safe on the silence.
"""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ScanSafetyNode(Node):
    """Zero forward velocity when the LiDAR sees an obstacle ahead."""

    def __init__(self):
        super().__init__('scan_safety')
        self.declare_parameter('sector_half_angle_deg', 30.0)
        self.declare_parameter('forward_offset_deg', 0.0)
        self.declare_parameter('stop_distance_m', 0.5)
        self.declare_parameter('min_range_m', 0.06)
        self.declare_parameter('scan_timeout_s', 0.5)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('control_period_s', 0.05)
        self._half = math.radians(
            self.get_parameter('sector_half_angle_deg').value)
        self._offset = math.radians(
            self.get_parameter('forward_offset_deg').value)
        self._stop = float(self.get_parameter('stop_distance_m').value)
        self._min_range = float(self.get_parameter('min_range_m').value)
        self._timeout = float(self.get_parameter('scan_timeout_s').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)
        self._period = float(self.get_parameter('control_period_s').value)

        self._scan = None
        self._scan_time = None
        self._cmd = None
        self._cmd_time = None
        self._pub_time = None
        self._sector = None

        self._pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self._obstacle_pub = self.create_publisher(Bool, '~/obstacle', 10)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_in', self._on_cmd, 10)
        self.create_timer(self._period, self._on_tick)
        self.get_logger().info(
            'scan_safety up: forward %.0f deg +/-%.0f deg, stop %.2f m, '
            'regating at %.0f Hz, command TTL %.2f s'
            % (math.degrees(self._offset), math.degrees(self._half),
               self._stop, 1.0 / self._period, self._cmd_timeout))

    def _on_scan(self, msg):
        self._scan = msg
        self._scan_time = self.get_clock().now()

    def _scan_fresh(self):
        if self._scan is None or self._scan_time is None:
            return False
        age = (self.get_clock().now() - self._scan_time).nanoseconds * 1e-9
        return age <= self._timeout

    def _sector_indices(self, scan):
        """Return the forward-sector indices, cached per scan geometry.

        Recomputed only when the scan geometry changes; the gate now runs at
        ``control_period_s`` rather than once per command, so re-deriving the
        bearing of all ~720 beams on every tick is pure waste.
        """
        key = (scan.angle_min, scan.angle_increment, len(scan.ranges))
        if self._sector is None or self._sector[0] != key:
            indices = []
            for i in range(len(scan.ranges)):
                angle = scan.angle_min + i * scan.angle_increment
                # Wrapped angular distance from robot-forward, so the sector
                # works at any mount offset (this C1 is a 180 deg-yaw mount).
                delta = math.atan2(math.sin(angle - self._offset),
                                   math.cos(angle - self._offset))
                if -self._half <= delta <= self._half:
                    indices.append(i)
            self._sector = (key, indices)
        return self._sector[1]

    def _forward_min(self):
        scan = self._scan
        best = math.inf
        for i in self._sector_indices(scan):
            rng = scan.ranges[i]
            if self._min_range <= rng <= scan.range_max:
                best = min(best, rng)
        return best

    def _on_cmd(self, cmd):
        """Hold the raw command; the timer decides what reaches the chassis."""
        self._cmd = cmd
        self._cmd_time = self.get_clock().now()
        # Gate and forward immediately so a fresh command is not delayed by up
        # to one tick; the timer then keeps re-gating it.
        self._publish(cmd)

    def _cmd_expired(self):
        if self._cmd_time is None:
            return True
        age = (self.get_clock().now() - self._cmd_time).nanoseconds * 1e-9
        return age > self._cmd_timeout

    def _on_tick(self):
        if self._cmd is None:
            return
        if self._cmd_expired():
            # Command source went quiet: stop once, then stay silent so the
            # firmware cmd_vel watchdog still sees the silence and fails safe.
            self._cmd = None
            self._cmd_time = None
            self._pub.publish(Twist())
            self._obstacle_pub.publish(Bool())
            return
        # A source that already publishes at least this fast (Nav2) has been
        # gated on arrival; re-gating it here too would only double the rate
        # seen by the chassis.
        since_pub = (
            self.get_clock().now() - self._pub_time).nanoseconds * 1e-9
        if since_pub < self._period:
            return
        self._publish(self._cmd)

    def _publish(self, cmd):
        blocked = False
        if cmd.linear.x > 0.0:
            if not self._scan_fresh():
                blocked = True
                self.get_logger().warn(
                    'no fresh scan -> blocking forward',
                    throttle_duration_sec=2.0)
            elif self._forward_min() < self._stop:
                blocked = True

        out = Twist()
        out.linear.x = 0.0 if blocked else cmd.linear.x
        out.linear.y = cmd.linear.y
        out.linear.z = cmd.linear.z
        out.angular.x = cmd.angular.x
        out.angular.y = cmd.angular.y
        out.angular.z = cmd.angular.z
        self._pub.publish(out)
        self._pub_time = self.get_clock().now()

        status = Bool()
        status.data = blocked
        self._obstacle_pub.publish(status)


def main(args=None):
    """Spin the scan-safety node."""
    rclpy.init(args=args)
    node = ScanSafetyNode()
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
