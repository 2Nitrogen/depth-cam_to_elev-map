"""state_estimator_node.

Subscribes to an IMU topic (typically the Madgwick-filtered /imu/data, which
has .orientation populated), feeds samples into a StateEstimatorBase
implementation, and publishes:
  - tf: odom -> base_link, with `odom` treated as gravity-aligned (REP-103
        world-up).
  - nav_msgs/Odometry on a configurable topic.

Default estimator is `gravity_from_imu` which extracts roll/pitch from the
upstream quaternion, strips yaw, and forces translation = [0,0,0]. Drop in
new estimators by subclassing StateEstimatorBase and extending the
`_make_estimator` factory.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import tf2_ros

from .estimators import (
    GravityFromImuQuaternion,
    IdentityStateEstimator,
    Pose,
    StateEstimatorBase,
)
from .imu_utils import imu_msg_to_arrays, stamp_to_sec


def _make_estimator(name: str) -> StateEstimatorBase:
    if name == 'identity':
        return IdentityStateEstimator()
    if name == 'gravity_from_imu':
        return GravityFromImuQuaternion()
    raise ValueError(
        f"Unknown estimator_type '{name}'. "
        f"Valid options: 'identity', 'gravity_from_imu'. "
        f"Add new ones in _make_estimator."
    )


class StateEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__('state_estimator_node')

        # --- parameters ---
        self.declare_parameter('imu_mode', 'combined')          # combined | split
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('accel_topic', '/camera/camera/accel/sample')
        self.declare_parameter('gyro_topic', '/camera/camera/gyro/sample')
        self.declare_parameter('estimator_type', 'gravity_from_imu')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'camera_imu_optical_frame')

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_odometry', True)
        self.declare_parameter('odom_topic', '/state_estimator/odometry')
        self.declare_parameter('publish_rate_hz', 50.0)

        self.declare_parameter('queue_size', 100)
        self.declare_parameter('debug_log', False)

        p = self.get_parameter
        self.imu_mode = str(p('imu_mode').value)
        self.imu_topic = str(p('imu_topic').value)
        self.accel_topic = str(p('accel_topic').value)
        self.gyro_topic = str(p('gyro_topic').value)
        self.estimator_type = str(p('estimator_type').value)

        self.odom_frame = str(p('odom_frame').value)
        self.base_frame = str(p('base_frame').value)
        self.imu_frame = str(p('imu_frame').value)

        self.publish_tf = bool(p('publish_tf').value)
        self.publish_odom = bool(p('publish_odometry').value)
        self.odom_topic = str(p('odom_topic').value)
        self.publish_rate_hz = float(p('publish_rate_hz').value)

        self.queue_size = int(p('queue_size').value)
        self.debug_log = bool(p('debug_log').value)

        # --- estimator ---
        self.estimator: StateEstimatorBase = _make_estimator(self.estimator_type)

        # --- IMU rate stats ---
        self._imu_count_window = 0
        self._imu_window_start: Optional[float] = None

        # --- TF: needed when the estimator requires the static imu->base
        # rotation (e.g. GravityFromImuQuaternion composes q_world_imu with
        # q_imu_base). Look up lazily on every IMU msg until success, then
        # cache. This is robust to startup ordering — the RealSense wrapper
        # may not have published its camera_link->imu_optical_frame static
        # TF yet when this node starts.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._needs_imu_to_base = hasattr(self.estimator, 'set_imu_to_base')
        self._imu_to_base_done = False

        # --- pubs ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        else:
            self.odom_pub = None

        # --- subs (sensor QoS) ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.queue_size,
        )
        if self.imu_mode == 'combined':
            self.create_subscription(
                Imu, self.imu_topic, self._on_imu, sensor_qos,
            )
            self.get_logger().info(f'IMU mode=combined, subscribed to {self.imu_topic}')
        elif self.imu_mode == 'split':
            self.create_subscription(
                Imu, self.accel_topic, self._on_accel, sensor_qos,
            )
            self.create_subscription(
                Imu, self.gyro_topic, self._on_gyro, sensor_qos,
            )
            self.get_logger().info(
                f'IMU mode=split, subscribed to {self.accel_topic} and {self.gyro_topic}'
            )
        else:
            raise ValueError(
                f"imu_mode must be 'combined' or 'split', got '{self.imu_mode}'"
            )

        # --- publish timer ---
        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.create_timer(period, self._on_publish_tick)

        self.get_logger().info(
            f'estimator_type={self.estimator_type} '
            f'odom_frame={self.odom_frame} base_frame={self.base_frame} '
            f'publish_rate={self.publish_rate_hz}Hz'
        )

    # ----- TF helper -----
    def _try_inject_imu_to_base(self) -> None:
        """Look up the static IMU->base rotation and forward it to the
        estimator. Tried on every IMU callback until success.

        Until success the estimator falls back to identity orientation
        (no gravity correction). A throttled WARN every 10 s tells the
        user the chain is incomplete; usual cause is a missing URDF /
        robot_state_publisher or `publish_camera_mount:=false` without
        an external publisher of base_link -> camera_link.
        """
        if not self._needs_imu_to_base or self._imu_to_base_done:
            return
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self.imu_frame,       # target: express vectors in imu coords
                self.base_frame,      # source: from base coords
                rclpy.time.Time(),    # latest available
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(
                f'TF lookup {self.base_frame} -> {self.imu_frame} not '
                f'available yet ({type(e).__name__}: {e}). Estimator is '
                f'running with identity orientation until the chain is '
                f'complete. Check that base_link -> camera_link is '
                f'published (URDF / robot_state_publisher, or the '
                f'bringup launch arg publish_camera_mount).',
                throttle_duration_sec=10.0,
            )
            return
        q = np.array([
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ], dtype=np.float64)
        self.estimator.set_imu_to_base(q)
        self._imu_to_base_done = True
        self.get_logger().info(
            f'Loaded static rotation q({self.base_frame} -> {self.imu_frame} '
            f'coords) into estimator: '
            f'q(xyzw)=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})'
        )

    # ----- IMU callbacks -----
    def _on_imu(self, msg: Imu) -> None:
        self._try_inject_imu_to_base()
        stamp_sec, acc, gyr, orient = imu_msg_to_arrays(msg)
        self.estimator.update_imu(stamp_sec, acc, gyr, orient)
        self._tally_imu(stamp_sec)

    def _on_accel(self, msg: Imu) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        acc = np.array(
            [msg.linear_acceleration.x,
             msg.linear_acceleration.y,
             msg.linear_acceleration.z],
            dtype=np.float64,
        )
        self.estimator.update_accel(stamp_sec, acc)
        self._tally_imu(stamp_sec)

    def _on_gyro(self, msg: Imu) -> None:
        stamp_sec = stamp_to_sec(msg.header.stamp)
        gyr = np.array(
            [msg.angular_velocity.x,
             msg.angular_velocity.y,
             msg.angular_velocity.z],
            dtype=np.float64,
        )
        self.estimator.update_gyro(stamp_sec, gyr)
        self._tally_imu(stamp_sec)

    def _tally_imu(self, stamp_sec: float) -> None:
        if not self.debug_log:
            return
        if self._imu_window_start is None:
            self._imu_window_start = stamp_sec
            self._imu_count_window = 0
        self._imu_count_window += 1
        elapsed = stamp_sec - self._imu_window_start
        if elapsed >= 1.0:
            hz = self._imu_count_window / elapsed
            self.get_logger().debug(f'IMU rx rate ~ {hz:.1f} Hz')
            self._imu_window_start = stamp_sec
            self._imu_count_window = 0

    # ----- publish tick -----
    def _on_publish_tick(self) -> None:
        # self.get_clock() respects the use_sim_time parameter:
        #   - use_sim_time=False -> system time (live camera scenario)
        #   - use_sim_time=True  -> /clock topic (rosbag --clock scenario)
        # Both the timer firing and this stamp must come from the same
        # source so the TF buffer of downstream consumers is internally
        # consistent. Do not mix in time.time() or rclpy.clock.Clock()
        # constructors here — they bypass use_sim_time.
        now = self.get_clock().now()
        self.estimator.predict(now.nanoseconds * 1e-9)
        pose = self.estimator.get_pose()

        if self.publish_tf:
            self.tf_broadcaster.sendTransform(self._pose_to_tf(pose, now))

        if self.odom_pub is not None:
            self.odom_pub.publish(self._pose_to_odom(pose, now))

    def _pose_to_tf(self, pose: Pose, now) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(pose.position[0])
        t.transform.translation.y = float(pose.position[1])
        t.transform.translation.z = float(pose.position[2])
        t.transform.rotation.x = float(pose.orientation[0])
        t.transform.rotation.y = float(pose.orientation[1])
        t.transform.rotation.z = float(pose.orientation[2])
        t.transform.rotation.w = float(pose.orientation[3])
        return t

    def _pose_to_odom(self, pose: Pose, now) -> Odometry:
        twist = self.estimator.get_twist()
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(pose.position[0])
        msg.pose.pose.position.y = float(pose.position[1])
        msg.pose.pose.position.z = float(pose.position[2])
        msg.pose.pose.orientation.x = float(pose.orientation[0])
        msg.pose.pose.orientation.y = float(pose.orientation[1])
        msg.pose.pose.orientation.z = float(pose.orientation[2])
        msg.pose.pose.orientation.w = float(pose.orientation[3])
        msg.twist.twist.linear.x = float(twist.linear[0])
        msg.twist.twist.linear.y = float(twist.linear[1])
        msg.twist.twist.linear.z = float(twist.linear[2])
        msg.twist.twist.angular.x = float(twist.angular[0])
        msg.twist.twist.angular.y = float(twist.angular[1])
        msg.twist.twist.angular.z = float(twist.angular[2])
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
