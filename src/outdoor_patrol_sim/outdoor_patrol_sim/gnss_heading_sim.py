# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Fill the gaps between the gz NavSat sensor and the UM982 driver.

The simulated GNSS is missing three things the real UM982 stack provides, and
all three are load-bearing for the global-localization chain:

1. **Covariance.** ``gz.msgs.NavSat`` carries no covariance, so the bridged
   fix arrives with an all-zero matrix. ``confidence_gate`` grades the fix on
   its reported horizontal sigma and the global EKF weights it by covariance,
   so a zero matrix means "infinitely trustworthy" — the filter would lock
   onto GNSS and ignore everything else. This node stamps the fix with the
   sigma it actually applies below.

2. **Metric noise.** gz-sim's navsat ``<position_sensing>`` noise is added to
   the latitude / longitude *in degrees*, so a plausible-looking 0.02 puts the
   fix kilometres away. The sensor is therefore configured noise-free and the
   noise is added here, in metres, converted to degrees at the fix's own
   latitude.

3. **Dual-antenna heading.** The gz NavSat sensor is single-antenna and has no
   heading output, while ``ekf_global`` fuses ``/gnss/heading`` as its only
   absolute-yaw source. This node synthesises that Imu message from Gazebo's
   ground-truth pose. It matches ``heading_to_imu``'s output contract (yaw-only
   Imu, ``frame_id: base_link``, angular velocity + acceleration marked
   unavailable) — the ``yaw_offset`` / antenna-baseline convention lives in the
   real adapter and is deliberately NOT re-simulated here.
"""

import math
import random

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

# WGS84 mean radius, for the metres <-> degrees conversion.
_EARTH_RADIUS_M = 6371000.0


def _yaw_from_quaternion(q: Quaternion) -> float:
    """Extract the ENU yaw (rad) from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GnssSim(Node):
    """Covariance-stamped fix + synthetic dual-antenna heading."""

    def __init__(self) -> None:
        super().__init__('gnss_sim')

        self.declare_parameter('fix_input_topic', '/gnss/fix_sim')
        self.declare_parameter('fix_output_topic', '/um982_driver/fix')
        self.declare_parameter('truth_topic', '/odom_truth')
        self.declare_parameter('heading_output_topic', '/gnss/heading')
        self.declare_parameter('heading_frame_id', 'base_link')
        # Applied here in metres (the gz sensor is left noise-free) and
        # reported as the fix covariance, at RTK-fixed scale so the
        # confidence gate (max_horizontal_sigma_m = 0.05) passes it.
        self.declare_parameter('horizontal_stddev_m', 0.02)
        self.declare_parameter('vertical_stddev_m', 0.04)
        # Mirrors heading_to_imu's yaw_stddev_deg default.
        self.declare_parameter('yaw_stddev_deg', 1.0)
        self.declare_parameter('heading_rate_hz', 5.0)

        self._h_var = self.get_parameter(
            'horizontal_stddev_m').value ** 2
        self._v_var = self.get_parameter('vertical_stddev_m').value ** 2
        self._h_stddev = self.get_parameter('horizontal_stddev_m').value
        self._v_stddev = self.get_parameter('vertical_stddev_m').value
        self._yaw_stddev = math.radians(
            self.get_parameter('yaw_stddev_deg').value)
        self._heading_frame = self.get_parameter('heading_frame_id').value

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.RELIABLE

        self._fix_pub = self.create_publisher(
            NavSatFix, self.get_parameter('fix_output_topic').value,
            sensor_qos)
        self._heading_pub = self.create_publisher(
            Imu, self.get_parameter('heading_output_topic').value, sensor_qos)

        self.create_subscription(
            NavSatFix, self.get_parameter('fix_input_topic').value,
            self._on_fix, sensor_qos)
        self.create_subscription(
            Odometry, self.get_parameter('truth_topic').value,
            self._on_truth, sensor_qos)

        self._latest_yaw = None
        period = 1.0 / float(self.get_parameter('heading_rate_hz').value)
        self.create_timer(period, self._publish_heading)

        self.get_logger().info(
            'gnss_sim up: fix sigma_h=%.3f m, heading sigma=%.2f deg'
            % (math.sqrt(self._h_var), math.degrees(self._yaw_stddev)))

    def _on_fix(self, msg: NavSatFix) -> None:
        # Metres -> degrees at this latitude (WGS84 meridian / parallel arc
        # lengths; a spherical approximation is well inside the noise here).
        lat_m_per_deg = _EARTH_RADIUS_M * math.pi / 180.0
        lon_m_per_deg = lat_m_per_deg * max(
            math.cos(math.radians(msg.latitude)), 1e-6)

        msg.latitude += random.gauss(0.0, self._h_stddev) / lat_m_per_deg
        msg.longitude += random.gauss(0.0, self._h_stddev) / lon_m_per_deg
        msg.altitude += random.gauss(0.0, self._v_stddev)

        # DIAGONAL_KNOWN so navsat_transform / the EKF use these numbers
        # rather than falling back on their own defaults.
        msg.position_covariance = [
            self._h_var, 0.0, 0.0,
            0.0, self._h_var, 0.0,
            0.0, 0.0, self._v_var,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        # The gz sensor leaves status at its default; the gate drops NO_FIX.
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        self._fix_pub.publish(msg)

    def _on_truth(self, msg: Odometry) -> None:
        self._latest_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _publish_heading(self) -> None:
        if self._latest_yaw is None:
            return

        yaw = self._latest_yaw + random.gauss(0.0, self._yaw_stddev)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._heading_frame
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        msg.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, self._yaw_stddev ** 2,
        ]
        # -1 in element 0 is the REP-145 "this field is unavailable" flag.
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self._heading_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssSim()
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
