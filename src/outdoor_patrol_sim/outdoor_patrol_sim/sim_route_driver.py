# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Drive the ground-truth road centerline, for a repeatable teach pass.

Simulation only, and deliberately dumb: it follows
``worlds/patrol_road_centerline.yaml`` -- the generator's own output -- using
Gazebo ground truth (``/odom_truth``) rather than anything the localization
stack produces.

That separation is what keeps the teach pass honest. The recorder under test
still records from the real GNSS -> confidence_gate -> navsat_transform -> EKF
chain, so nothing about the measurement is short-circuited; this node only
removes the human from the loop so two teach passes are the same teach pass.

Commands go to ``/cmd_vel_raw``, so the M3 forward brake stays in the path
exactly as it does for teleop. With obstacles in the world the brake WILL stop
this node -- it has no avoidance of its own. Teach on ``patrol_road.sdf``, the
clear world.

Publishes ``~/finished`` (std_msgs/Bool, transient-local) once the requested
number of laps is done, so a test harness can wait on an event instead of a
stopwatch.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import Bool

import yaml


def _yaw_of(orientation) -> float:
    q = orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimRouteDriver(Node):
    """Pure pursuit on the ground-truth centerline."""

    def __init__(self) -> None:
        super().__init__('sim_route_driver')

        self.declare_parameter('centerline_path', '')
        self.declare_parameter('truth_topic', '/odom_truth')
        self.declare_parameter('cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('speed_ms', 0.8)
        self.declare_parameter('lookahead_m', 1.5)
        self.declare_parameter('max_angular_rads', 0.67)
        self.declare_parameter('laps', 1.0)
        self.declare_parameter('control_period_s', 0.05)
        # Drive the first metres slowly: the chassis starts stationary and a
        # full-speed step makes the wheels slip, which shows up as a wheel-
        # odometry transient the EKF then has to reject.
        self.declare_parameter('start_ramp_m', 2.0)

        path = self.get_parameter('centerline_path').value
        if not path:
            raise ValueError('centerline_path is required')
        self._load(path)

        self._speed = float(self.get_parameter('speed_ms').value)
        self._lookahead = float(self.get_parameter('lookahead_m').value)
        self._max_omega = float(self.get_parameter('max_angular_rads').value)
        self._laps = float(self.get_parameter('laps').value)
        self._ramp = float(self.get_parameter('start_ramp_m').value)

        self._index = 0
        self._travelled = 0.0
        self._last_xy = None
        self._done = False

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._cmd = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, qos)
        self._finished = self.create_publisher(Bool, '~/finished', latched)
        self.create_subscription(
            Odometry, self.get_parameter('truth_topic').value,
            self._on_truth, qos)

        self._pose = None
        self.create_timer(
            float(self.get_parameter('control_period_s').value),
            self._control)

        self.get_logger().info(
            'sim_route_driver: %.1f m centerline, %.2f laps at %.2f m/s, '
            'look-ahead %.2f m'
            % (self._length, self._laps, self._speed, self._lookahead))

    def _load(self, path: str) -> None:
        with open(path) as handle:
            road = yaml.safe_load(handle)['road']
        self._points = [(float(s['x']), float(s['y']))
                        for s in road['samples']]
        self._length = float(road['length_m'])
        self._loop = bool(road['loop'])
        if self._loop and len(self._points) > 1:
            # The generator repeats s=0 as the closing sample; drop it so the
            # wraparound is not a zero-length step.
            self._points = self._points[:-1]

    def _on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, _yaw_of(msg.pose.pose.orientation))
        if self._last_xy is not None:
            self._travelled += math.hypot(p.x - self._last_xy[0],
                                          p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)

    def _advance(self, x: float, y: float) -> int:
        """Move the index to the closest point ahead, then out by look-ahead."""
        n = len(self._points)
        # Track the nearest point, searching only forward so the index cannot
        # jump backwards across a corner.
        best, best_d2 = self._index, None
        for step in range(0, 120):
            i = (self._index + step) % n
            px, py = self._points[i]
            d2 = (px - x) ** 2 + (py - y) ** 2
            if best_d2 is None or d2 < best_d2:
                best, best_d2 = i, d2
        self._index = best

        remaining = self._lookahead
        i = best
        while remaining > 0.0:
            j = (i + 1) % n
            ax, ay = self._points[i]
            bx, by = self._points[j]
            remaining -= math.hypot(bx - ax, by - ay)
            i = j
        return i

    def _control(self) -> None:
        if self._pose is None or self._done:
            return

        if self._travelled >= self._laps * self._length:
            self._finish()
            return

        x, y, yaw = self._pose
        tx, ty = self._points[self._advance(x, y)]

        # Look-ahead point in the body frame; pure-pursuit curvature.
        dx, dy = tx - x, ty - y
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        distance = max(math.hypot(local_x, local_y), 1e-3)
        curvature = 2.0 * local_y / (distance * distance)

        speed = self._speed
        if self._ramp > 0.0 and self._travelled < self._ramp:
            speed *= max(0.25, self._travelled / self._ramp)

        omega = curvature * speed
        if abs(omega) > self._max_omega:
            # Keep the geometry: slow down rather than under-steer.
            speed *= self._max_omega / abs(omega)
            omega = math.copysign(self._max_omega, omega)

        command = Twist()
        command.linear.x = speed
        command.angular.z = omega
        self._cmd.publish(command)

    def _finish(self) -> None:
        self._done = True
        self._cmd.publish(Twist())
        self._finished.publish(Bool(data=True))
        self.get_logger().info(
            'teach pass complete: %.1f m driven (%.2f laps)'
            % (self._travelled, self._travelled / self._length))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimRouteDriver()
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
