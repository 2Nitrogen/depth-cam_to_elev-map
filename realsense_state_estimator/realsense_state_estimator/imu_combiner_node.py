"""imu_combiner_node — combine split accel/gyro topics into a unified Imu.

When the RealSense ROS2 wrapper runs without `unite_imu_method:=2` (or
when a bag was recorded that way), the camera publishes
  /camera/camera/accel/sample   (sensor_msgs/Imu, accel only)
  /camera/camera/gyro/sample    (sensor_msgs/Imu, gyro only)
instead of a single combined /camera/camera/imu. Downstream consumers
that expect a combined IMU (imu_filter_madgwick reads `imu/data_raw`)
get no data, leaving the orientation estimate empty.

This node bridges that gap: it caches the latest accel sample and, on
every gyro callback, publishes a combined sensor_msgs/Imu on the
configured output topic with the current gyro's stamp + frame_id. This
is the equivalent of realsense2_camera's COPY mode (use latest accel);
LINEAR_INTERP would be more accurate but is unnecessary for Madgwick,
which uses accel only as a gravity reference.

Safe to run when /camera/camera/imu is already being published, IF you
remap output_topic elsewhere — otherwise two publishers on the same
topic will produce duplicate messages.
"""
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class ImuCombinerNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_combiner_node')

        self.declare_parameter('accel_topic', '/camera/camera/accel/sample')
        self.declare_parameter('gyro_topic', '/camera/camera/gyro/sample')
        self.declare_parameter('output_topic', '/camera/camera/imu')
        self.declare_parameter('queue_size', 100)

        accel_topic = str(self.get_parameter('accel_topic').value)
        gyro_topic = str(self.get_parameter('gyro_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        queue_size = int(self.get_parameter('queue_size').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )

        self._latest_accel: Optional[Imu] = None
        self._published = 0
        self._dropped_no_accel = 0

        self.create_subscription(Imu, accel_topic, self._on_accel, sensor_qos)
        self.create_subscription(Imu, gyro_topic, self._on_gyro, sensor_qos)
        self.pub = self.create_publisher(Imu, output_topic, queue_size)

        self.get_logger().info(
            f'imu_combiner: accel={accel_topic} + gyro={gyro_topic} '
            f'-> {output_topic} (COPY-mode pairing on gyro stamps)'
        )

    def _on_accel(self, msg: Imu) -> None:
        self._latest_accel = msg

    def _on_gyro(self, msg: Imu) -> None:
        if self._latest_accel is None:
            self._dropped_no_accel += 1
            if self._dropped_no_accel == 1 or self._dropped_no_accel % 200 == 0:
                self.get_logger().warning(
                    f'No accel sample cached yet; dropped {self._dropped_no_accel} '
                    f'gyro msgs. Check that {self.get_parameter("accel_topic").value} '
                    f'is being published.'
                )
            return

        out = Imu()
        # gyro is the higher-rate signal; use its stamp + frame as the primary.
        out.header = msg.header
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = self._latest_accel.linear_acceleration
        out.linear_acceleration_covariance = \
            self._latest_accel.linear_acceleration_covariance
        # Mark orientation as unset (sensor_msgs/Imu convention: -1 in cov[0]).
        # Madgwick downstream will fill it.
        out.orientation_covariance = [-1.0] + [0.0] * 8
        self.pub.publish(out)
        self._published += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuCombinerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
