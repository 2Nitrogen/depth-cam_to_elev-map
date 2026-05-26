"""imu_combiner_node — combine split accel/gyro topics into a unified Imu.

When the RealSense ROS2 wrapper runs without `unite_imu_method:=2` (or
when a bag was recorded that way), the camera publishes
  /camera/camera/accel/sample   (sensor_msgs/Imu, accel only)
  /camera/camera/gyro/sample    (sensor_msgs/Imu, gyro only)
instead of a single combined /camera/camera/imu. Downstream consumers
that expect a combined IMU (imu_filter_madgwick reads `imu/data_raw`)
get no data, leaving the orientation estimate empty.

This node bridges that gap by caching the latest accel sample and, on
every gyro callback, publishing a combined sensor_msgs/Imu on the
configured output topic with the current gyro's stamp + frame_id. This
is the equivalent of realsense2_camera's COPY mode (use latest accel);
LINEAR_INTERP would be more accurate but is unnecessary for Madgwick,
which uses accel only as a gravity reference.

# Adaptive activation

To stay safe in setups where the output topic is *already* being
published (e.g. wrapper started with `unite_imu_method:=2`, or a bag
that has the combined topic baked in), the node introspects the topic
periodically and goes passive when another publisher is present. This
removes the duplicate-publisher hazard and means the same launch
configuration works for any combination of split/combined sources
(live or replayed) without special-casing.
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
        # How often to re-check whether another publisher is on the output
        # topic. 1 Hz is fast enough to catch wrapper start/stop events and
        # cheap enough to leave running.
        self.declare_parameter('mode_check_period_sec', 1.0)

        self._accel_topic = str(self.get_parameter('accel_topic').value)
        self._gyro_topic = str(self.get_parameter('gyro_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        queue_size = int(self.get_parameter('queue_size').value)
        mode_check_period = float(
            self.get_parameter('mode_check_period_sec').value
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )

        self._latest_accel: Optional[Imu] = None
        self._published = 0
        self._dropped_no_accel = 0

        # Activation state:
        #   None  - undecided, no log emitted yet (first check pending)
        #   True  - active, combining and publishing
        #   False - passive, another publisher owns the output topic
        self._active: Optional[bool] = None
        self._other_publisher_name: Optional[str] = None

        self.create_subscription(Imu, self._accel_topic, self._on_accel, sensor_qos)
        self.create_subscription(Imu, self._gyro_topic, self._on_gyro, sensor_qos)
        self.pub = self.create_publisher(Imu, self._output_topic, queue_size)

        self.get_logger().info(
            f'imu_combiner: accel={self._accel_topic} + gyro={self._gyro_topic} '
            f'-> {self._output_topic} (adaptive: goes passive if another '
            f'publisher claims the output topic)'
        )

        # Run an immediate check and then schedule recurring ones. The
        # immediate check sets self._active so on_gyro callbacks coming
        # in right away know whether to publish.
        self._update_mode()
        self._mode_timer = self.create_timer(mode_check_period, self._update_mode)

    def _update_mode(self) -> None:
        """Inspect the output topic's publishers and decide active/passive.

        Excludes our own publisher from the count. If another publisher is
        present we go passive; if it disappears we go active again. The
        first call also sets the initial state (None -> True/False).
        """
        try:
            infos = self.get_publishers_info_by_topic(self._output_topic)
        except (RuntimeError, NotImplementedError) as e:
            # If introspection isn't available, default to active rather
            # than silently doing nothing.
            self.get_logger().warning(
                f'Could not introspect publishers of {self._output_topic} '
                f'({e}); defaulting to active.'
            )
            if self._active is not True:
                self._active = True
            return

        my_name = self.get_name()
        my_ns = self.get_namespace()
        others = [
            info for info in infos
            if not (info.node_name == my_name and info.node_namespace == my_ns)
        ]

        if others:
            new_active = False
            other_name = others[0].node_name
        else:
            new_active = True
            other_name = None

        if new_active == self._active:
            # State unchanged, nothing to log.
            self._other_publisher_name = other_name
            return

        if new_active:
            if self._active is False:
                self.get_logger().info(
                    f'Other publisher on {self._output_topic} disappeared '
                    f'(was {self._other_publisher_name!r}); resuming '
                    f'combiner output.'
                )
            else:
                # First check (was None)
                self.get_logger().info(
                    f'No other publisher on {self._output_topic}; '
                    f'imu_combiner ACTIVE (combining accel + gyro).'
                )
        else:
            if self._active is True:
                self.get_logger().info(
                    f'New publisher on {self._output_topic} detected '
                    f'({other_name!r}); imu_combiner going PASSIVE to '
                    f'avoid duplicates.'
                )
            else:
                # First check (was None)
                self.get_logger().info(
                    f'Another publisher already on {self._output_topic} '
                    f'({other_name!r}); imu_combiner PASSIVE (no combining '
                    f'needed).'
                )

        self._active = new_active
        self._other_publisher_name = other_name

    def _on_accel(self, msg: Imu) -> None:
        self._latest_accel = msg

    def _on_gyro(self, msg: Imu) -> None:
        if self._active is not True:
            return  # passive (or pre-decision)

        if self._latest_accel is None:
            self._dropped_no_accel += 1
            if self._dropped_no_accel == 1 or self._dropped_no_accel % 200 == 0:
                self.get_logger().warning(
                    f'No accel sample cached yet; dropped {self._dropped_no_accel} '
                    f'gyro msgs. Check that {self._accel_topic} is being '
                    f'published.'
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
