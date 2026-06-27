# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Adapt the UM982 dual-antenna heading to a yaw-only sensor_msgs/Imu.

The receiver publishes its dual-antenna baseline heading as a
``geometry_msgs/QuaternionStamped``, which neither robot_localization's EKF
nor ``navsat_transform_node`` can consume directly. This node republishes it
as a ``sensor_msgs/Imu`` carrying ONLY an absolute yaw orientation (angular
velocity and linear acceleration are marked unavailable per the Imu
convention, ``covariance[0] = -1``), so it can stand in for an IMU's heading
until the real IMU lands at M2 (interim per ADR-012).

A single ``yaw_offset`` (plus ``invert``) maps the receiver's heading
convention + antenna mounting into a REP-103 yaw (0 = East, CCW positive):

    yaw_rep103 = yaw_offset + sign * yaw_in

TBD: set ``yaw_offset`` / ``invert`` once the heading convention and the
antenna-baseline mounting angle are confirmed (integration plan items 2/3).
Until then the defaults pass the receiver yaw through unchanged.
"""
import math

import rclpy
from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Planar yaw (rad) from a full quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class HeadingToImu(Node):
    """Republish a QuaternionStamped heading as a yaw-only Imu."""

    def __init__(self) -> None:
        super().__init__('heading_to_imu')
        self.declare_parameter('input_topic', '/um982_driver/heading')
        self.declare_parameter('output_topic', '/gnss/heading')
        self.declare_parameter('frame_id', 'base_link')
        # TBD: receiver convention (compass->ENU) + antenna mount, radians.
        self.declare_parameter('yaw_offset', 0.0)
        # TBD: set true if the receiver heading increases clockwise.
        self.declare_parameter('invert', False)
        # 1-sigma heading uncertainty (deg) -> orientation yaw covariance.
        self.declare_parameter('yaw_stddev_deg', 1.0)

        self._frame = self.get_parameter('frame_id').value
        self._yaw_offset = float(self.get_parameter('yaw_offset').value)
        self._sign = -1.0 if self.get_parameter('invert').value else 1.0
        sd = math.radians(float(self.get_parameter('yaw_stddev_deg').value))
        self._yaw_var = sd * sd

        self._pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            QuaternionStamped, self.get_parameter('input_topic').value,
            self._cb, 10)

    def _cb(self, msg: QuaternionStamped) -> None:
        q = msg.quaternion
        yaw = self._yaw_offset + self._sign * _yaw_from_quat(q.x, q.y, q.z, q.w)

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame
        out.orientation.z = math.sin(yaw / 2.0)
        out.orientation.w = math.cos(yaw / 2.0)
        # Only yaw is observed; roll/pitch get a large variance.
        out.orientation_covariance = [
            1.0e6, 0.0, 0.0,
            0.0, 1.0e6, 0.0,
            0.0, 0.0, self._yaw_var,
        ]
        # Angular velocity and linear acceleration are unavailable: the
        # leading -1 tells consumers to ignore them entirely.
        out.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0]
        out.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0]
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingToImu()
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
