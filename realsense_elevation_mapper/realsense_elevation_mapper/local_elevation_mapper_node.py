"""Local elevation mapper node.

Subscribes to a RealSense PointCloud2 topic, transforms to a target frame,
crops to a ROI, accumulates the last k frames, and publishes:
  - the accumulated point cloud
  - an elevation grid as PointCloud2 (cell centers + mean z)
"""
from __future__ import annotations

from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2

import tf2_ros

from .elevation_grid import compute_mean_elevation, grid_to_points, make_grid_spec
from .pointcloud_utils import crop_roi, pointcloud2_to_xyz, xyz_to_pointcloud2
from .tf_utils import transform_points


class LocalElevationMapperNode(Node):
    def __init__(self) -> None:
        super().__init__('local_elevation_mapper_node')

        # --- parameters ---
        self.declare_parameter('input_cloud_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('accumulated_cloud_topic', '/local_elevation_map/accumulated_points')
        self.declare_parameter('elevation_cloud_topic', '/local_elevation_map/points')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('k_frames', 5)

        self.declare_parameter('map_length_x', 2.0)
        self.declare_parameter('map_length_y', 2.0)
        self.declare_parameter('resolution', 0.02)

        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 2.0)
        self.declare_parameter('y_min', -1.0)
        self.declare_parameter('y_max', 1.0)
        self.declare_parameter('z_min', -0.5)
        self.declare_parameter('z_max', 1.0)

        self.declare_parameter('publish_accumulated_cloud', True)
        self.declare_parameter('publish_elevation_cloud', True)

        self.declare_parameter('min_points_per_cell', 1)
        self.declare_parameter('cloud_queue_size', 5)
        self.declare_parameter('tf_timeout_sec', 0.1)
        # When true, use the latest available TF instead of msg.header.stamp.
        # Useful when replaying a bag whose message stamps don't match /clock
        # (e.g. RealSense sensor hardware time vs bag record time).
        self.declare_parameter('use_latest_tf', True)

        p = self.get_parameter
        self.input_cloud_topic = p('input_cloud_topic').value
        self.accumulated_cloud_topic = p('accumulated_cloud_topic').value
        self.elevation_cloud_topic = p('elevation_cloud_topic').value
        self.target_frame = p('target_frame').value
        self.k_frames = int(p('k_frames').value)

        self.x_min = float(p('x_min').value)
        self.x_max = float(p('x_max').value)
        self.y_min = float(p('y_min').value)
        self.y_max = float(p('y_max').value)
        self.z_min = float(p('z_min').value)
        self.z_max = float(p('z_max').value)
        self.resolution = float(p('resolution').value)

        self.publish_accumulated_cloud = bool(p('publish_accumulated_cloud').value)
        self.publish_elevation_cloud = bool(p('publish_elevation_cloud').value)
        self.min_points_per_cell = int(p('min_points_per_cell').value)
        self.cloud_queue_size = int(p('cloud_queue_size').value)
        self.tf_timeout = Duration(seconds=float(p('tf_timeout_sec').value))
        self.use_latest_tf = bool(p('use_latest_tf').value)

        self.grid_spec = make_grid_spec(
            self.x_min, self.x_max, self.y_min, self.y_max, self.resolution,
        )
        self.cloud_buffer: deque[np.ndarray] = deque(maxlen=self.k_frames)

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- ROS pubs / subs ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.cloud_queue_size,
        )
        self.sub = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self._on_cloud, sensor_qos,
        )
        self.pub_accum = self.create_publisher(
            PointCloud2, self.accumulated_cloud_topic, 1,
        )
        self.pub_elev = self.create_publisher(
            PointCloud2, self.elevation_cloud_topic, 1,
        )

        self.get_logger().info(
            f'Subscribed to {self.input_cloud_topic} -> target_frame={self.target_frame} '
            f'grid={self.grid_spec.size_x}x{self.grid_spec.size_y}@{self.resolution}m '
            f'k_frames={self.k_frames}'
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        # TF lookup time policy.
        #
        # When use_latest_tf is True we look up the most recent available
        # transform (Time() = zero-time = "latest" in tf2) instead of the
        # one matching msg.header.stamp. This is the right default for two
        # common situations encountered with this stack:
        #
        # (a) Rosbag playback. The RealSense ROS2 wrapper stamps messages
        #     with sensor *hardware* time by default (i.e. unless launched
        #     with use_ros_time:=true at record time). When the bag is
        #     replayed with `ros2 bag play --clock`, /clock advances in
        #     bag-record (= recorder system) time, but each message's
        #     header.stamp still carries sensor hardware time. The two
        #     clocks are offset by a constant (often a few seconds), so
        #     looking up TF at msg.header.stamp always lands outside the
        #     buffer and fails as ExtrapolationException. There is no
        #     post-hoc fix in the bag — re-record with use_ros_time:=true
        #     if you need accurate per-cloud TF interpolation.
        #
        # (b) Identity / placeholder state estimator. While the estimator
        #     in realsense_state_estimator is IdentityStateEstimator, the
        #     odom -> base_link TF is constant. Per-frame interpolation
        #     gives nothing extra, so latest-TF is equivalent and avoids
        #     spurious extrapolation warnings during startup races.
        #
        # Set use_latest_tf=False once both hold:
        #   - a real estimator (Madgwick/EKF/VIO) is publishing meaningful
        #     time-varying motion on odom -> base_link, AND
        #   - cloud and TF live in the same time domain (live camera with
        #     use_ros_time:=true, or a bag recorded that way).
        lookup_time = Time() if self.use_latest_tf else msg.header.stamp
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id,
                lookup_time, timeout=self.tf_timeout,
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed ({msg.header.frame_id} -> {self.target_frame}): {e}'
            )
            return

        out_stamp = (
            self.get_clock().now().to_msg()
            if self.use_latest_tf else msg.header.stamp
        )

        pts = pointcloud2_to_xyz(msg)
        if pts.size == 0:
            return

        pts = transform_points(pts, tf)
        pts = crop_roi(
            pts,
            self.x_min, self.x_max,
            self.y_min, self.y_max,
            self.z_min, self.z_max,
        )

        self.cloud_buffer.append(pts)

        accumulated = (
            np.concatenate(list(self.cloud_buffer), axis=0)
            if len(self.cloud_buffer) > 0
            else np.zeros((0, 3), dtype=np.float32)
        )

        if self.publish_accumulated_cloud:
            self.pub_accum.publish(
                xyz_to_pointcloud2(accumulated, self.target_frame, out_stamp)
            )

        if self.publish_elevation_cloud:
            height_map, count_map = compute_mean_elevation(accumulated, self.grid_spec)
            elev_pts = grid_to_points(
                height_map, count_map, self.grid_spec,
                min_points_per_cell=self.min_points_per_cell,
            )
            self.pub_elev.publish(
                xyz_to_pointcloud2(elev_pts, self.target_frame, out_stamp)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalElevationMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
