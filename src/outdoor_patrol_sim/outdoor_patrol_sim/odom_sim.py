# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""Stamp the simulated wheel odometry with the firmware's covariances.

The gz DiffDrive plugin publishes `/odom` with an all-zero covariance matrix.
robot_localization reads a zero matrix as "infinitely trustworthy", so the
local EKF would ride the sim's perfect wheel odometry and effectively ignore
the GNSS and IMU — the exact opposite of how the stack behaves on the robot.

The real ESP32-S3 controller stamps a fixed diagonal (see the "Static odom
covariance diagonals" block in
`src/esp32-s3-uros-controller/firmware/main/uros_task.c`). This node applies
the same numbers to the simulated odometry so the EKF tuning that works in
sim also works in the field. Keep `config/odom_covariance.yaml` in lock-step
with the firmware.
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Index into the row-major 6x6 covariance matrix for each diagonal term.
_DIAG = (0, 7, 14, 21, 28, 35)


class OdomSim(Node):
    """Republish the gz odometry with firmware-matched covariance."""

    def __init__(self) -> None:
        super().__init__('odom_sim')

        self.declare_parameter('input_topic', '/odom_sim')
        self.declare_parameter('output_topic', '/odom')
        # Defaults mirror uros_task.c; override via the params file.
        self.declare_parameter(
            'pose_covariance_diagonal',
            [0.00096, 0.00060, 1.0e6, 1.0e6, 1.0e6, 0.00054])
        self.declare_parameter(
            'twist_covariance_diagonal',
            [0.001, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 0.003])

        self._pose_diag = list(
            self.get_parameter('pose_covariance_diagonal').value)
        self._twist_diag = list(
            self.get_parameter('twist_covariance_diagonal').value)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, qos)
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value,
            self._on_odom, qos)

    def _on_odom(self, msg: Odometry) -> None:
        for value, idx in zip(self._pose_diag, _DIAG):
            msg.pose.covariance[idx] = value
        for value, idx in zip(self._twist_diag, _DIAG):
            msg.twist.covariance[idx] = value
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomSim()
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
