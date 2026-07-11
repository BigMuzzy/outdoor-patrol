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
        self.declare_parameter('stop_distance_m', 0.5)
        self.declare_parameter('min_range_m', 0.06)
        self.declare_parameter('scan_timeout_s', 0.5)
        self._half = math.radians(
            self.get_parameter('sector_half_angle_deg').value)
        self._stop = float(self.get_parameter('stop_distance_m').value)
        self._min_range = float(self.get_parameter('min_range_m').value)
        self._timeout = float(self.get_parameter('scan_timeout_s').value)

        self._scan = None
        self._scan_time = None

        self._pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self._obstacle_pub = self.create_publisher(Bool, '~/obstacle', 10)
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_in', self._on_cmd, 10)
        self.get_logger().info(
            'scan_safety up: forward sector +/-%.0f deg, stop %.2f m'
            % (math.degrees(self._half), self._stop))

    def _on_scan(self, msg):
        self._scan = msg
        self._scan_time = self.get_clock().now()

    def _scan_fresh(self):
        if self._scan is None or self._scan_time is None:
            return False
        age = (self.get_clock().now() - self._scan_time).nanoseconds * 1e-9
        return age <= self._timeout

    def _forward_min(self):
        scan = self._scan
        best = math.inf
        angle = scan.angle_min
        for rng in scan.ranges:
            if -self._half <= angle <= self._half:
                if self._min_range <= rng <= scan.range_max:
                    best = min(best, rng)
            angle += scan.angle_increment
        return best

    def _on_cmd(self, cmd):
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
