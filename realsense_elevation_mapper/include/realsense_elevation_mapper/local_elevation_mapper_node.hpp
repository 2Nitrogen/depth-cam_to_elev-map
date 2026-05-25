#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "realsense_elevation_mapper/elevation_grid.hpp"

namespace realsense_elevation_mapper
{

// Per-callback data pipeline (three explicit stages):
//
//   in   : sensor_msgs/PointCloud2     N points, fields x,y,z (float32, m)
//                                      + rgb (float32 packed), frame =
//                                      camera_*_optical_frame, NaN allowed.
//   raw  : pcl::PointCloud<PointXYZ>   M <= N (NaN removed), optical frame.
//                                      Kept to compute per-point R from
//                                      raw-frame depth.
//   tf'd : pcl::PointCloud<PointXYZ>   M, frame = target_frame (default odom,
//                                      gravity-aligned per REP-103). Same
//                                      ordering as raw — index pairs line up.
//   meas : std::vector<HeightMeasurement>  M, bundles target-frame (x,y,z)
//                                      with R = compute_measurement_variance(raw[i]).
//   crop : same M' <= M, applied only when all six ROI bounds are
//          provided in YAML (otherwise inspection mode -> no crop).
//
//   ❶ bin   : std::vector<CellMeasurement>  K <= M', one entry per occupied
//             cell in the frame. (x_center, y_center) world coords + mean z
//             + mean R + raw point count. Implemented by
//             bin_frame_into_cells() — within-cell aggregation finishes here.
//   ❷ accum : deque<BinnedFrame> measurement_buffer_, the last k_frames
//             of bin results. Sliding window. Raw points are not kept.
//   ❸ fuse  : for each buffered frame, fuse_binned_frame() walks the cells
//             and performs exactly one Mahalanobis-gated Kalman update per
//             cell against the current MapLayers (cell-to-cell pairing).
//
//   out_a: sensor_msgs/PointCloud2 cell-centre representation of the
//          accumulated buffer (sum over frames of |cells|), frame = target_frame.
//   out_e: sensor_msgs/PointCloud2 <= size_x * size_y points (one per
//          occupied cell), (x,y) at cell centre, z = per-cell estimate,
//          intensity = sqrt(map estimate variance P).
//
// Frame semantics:
//   target_frame      : the gravity-aligned global frame the map lives in
//                       (default `odom`). Elevation z and the XY grid plane
//                       are expressed in this frame.
//   track_point_frame : the body frame the local map window follows
//                       (default `base_link`). When ROI bounds are set, they
//                       are interpreted as offsets relative to this frame's
//                       XY position in target_frame. Today translation is
//                       forced to zero by the state estimator, so the
//                       effective bounds equal the configured ones; once
//                       real odometry comes in, the window will follow the
//                       robot automatically.
//
// ROI / grid mode (chosen once at construction):
//   * roi_set_ == true : ROI crop active. Grid extent each callback =
//                        configured bounds shifted by the current
//                        target_frame -> track_point_frame translation.
//   * roi_set_ == false: inspection mode, no crop, grid extent rebuilt
//                        each frame from the accumulated cloud's actual
//                        XY extent so that every valid depth point shows
//                        up in the elevation cloud.
class LocalElevationMapperNode : public rclcpp::Node
{
public:
  LocalElevationMapperNode();

private:
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // parameters
  std::string input_cloud_topic_;
  std::string accumulated_cloud_topic_;
  std::string elevation_cloud_topic_;
  std::string target_frame_;       // gravity-aligned map frame
  std::string track_point_frame_;  // body frame the window follows
  int k_frames_{5};

  // ROI bounds. NaN sentinel means "not provided in YAML" -> inspection
  // mode (see class comment).
  double x_min_{std::numeric_limits<double>::quiet_NaN()};
  double x_max_{std::numeric_limits<double>::quiet_NaN()};
  double y_min_{std::numeric_limits<double>::quiet_NaN()};
  double y_max_{std::numeric_limits<double>::quiet_NaN()};
  double z_min_{std::numeric_limits<double>::quiet_NaN()};
  double z_max_{std::numeric_limits<double>::quiet_NaN()};
  double resolution_{0.02};

  bool publish_accumulated_cloud_{true};
  bool publish_elevation_cloud_{true};
  // Derived at construction: true iff all six ROI bounds are finite.
  bool roi_set_{false};
  int min_points_per_cell_{1};
  int cloud_queue_size_{5};
  double tf_timeout_sec_{0.1};
  bool use_latest_tf_{true};

  // Per-cell fusion tuning (see elevation_grid.hpp::FusionParams).
  FusionParams fusion_params_;
  // When true, log per-callback min/mean/max of the measurement variance
  // computed across the accumulated buffer (throttled).
  bool debug_measurement_variance_{false};
  // When true, layers are reset at the start of every elevation publish so
  // the map represents only the current accumulated buffer. When false,
  // layers persist across callbacks and stale cells are aged out by
  // `max_age_sec`. Stationary use case favors true; locomotion will favor
  // false once translation is wired in.
  bool enable_continuous_cleanup_{true};
  double max_age_sec_{2.0};

  // derived state. The grid spec itself is *not* a member — on_cloud
  // computes a fresh frame_spec each callback (so it can shift with the
  // track point in ROI mode or auto-fit in inspection mode).
  MapLayers layers_;
  // Last k_frames worth of *binned* frames (per-cell aggregates, not raw
  // points). One BinnedFrame per arrival. CellMeasurement stores world-frame
  // cell-centre coords so the fusion grid spec is free to differ from the
  // bin spec (matters when inspection mode auto-fits each callback).
  std::deque<BinnedFrame> measurement_buffer_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_accum_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elev_;
};

}  // namespace realsense_elevation_mapper
