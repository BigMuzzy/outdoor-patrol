# Copyright 2026 Outdoor Patrol Team
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 driver node for the WitMotion UM982 RTK GNSS/INS module.

This is a boilerplate skeleton. Protocol parsing (NMEA / Unicore binary,
heading, IMU passthrough) will be filled in once the datasheet and
manual are available.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64


class Um982DriverNode(Node):
    """Read serial frames from the UM982 and publish ROS messages."""

    def __init__(self) -> None:
        super().__init__('um982_driver')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('frame_id', 'gnss_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 10.0)

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baudrate').value
        self._frame_id = self.get_parameter('frame_id').value
        self._imu_frame_id = self.get_parameter('imu_frame_id').value

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.heading_pub = self.create_publisher(
            Float64, 'gnss/heading_deg', 10)

        # TODO: open serial port and start read loop
        self.get_logger().info(
            f'UM982 driver boilerplate started (port={self._port}, '
            f'baud={self._baud}). Protocol parsing not yet implemented.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Um982DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
