#include "realsense_elevation_mapper/local_elevation_mapper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>

#include "realsense_elevation_mapper/pointcloud_utils.hpp"

namespace realsense_elevation_mapper
{
namespace
{

MeasurementVarianceModel parse_mv_model(const std::string & s)
{
  if (s == "constant")      { return MeasurementVarianceModel::Constant; }
  if (s == "depth_squared") { return MeasurementVarianceModel::DepthSquared; }
  throw std::runtime_error(
    "Invalid measurement_variance_model '" + s +
    "'. Valid options: 'constant', 'depth_squared'.");
}

DepthSource parse_depth_source(const std::string & s)
{
  if (s == "raw_z")      { return DepthSource::RawZ; }
  if (s == "range_norm") { return DepthSource::RangeNorm; }
  throw std::runtime_error(
    "Invalid depth_source '" + s +
    "'. Valid options: 'raw_z', 'range_norm'.");
}

const char * mv_model_str(MeasurementVarianceModel m)
{
  switch (m) {
    case MeasurementVarianceModel::Constant:     return "constant";
    case MeasurementVarianceModel::DepthSquared: return "depth_squared";
  }
  return "?";
}

const char * depth_source_str(DepthSource d)
{
  switch (d) {
    case DepthSource::RawZ:      return "raw_z";
    case DepthSource::RangeNorm: return "range_norm";
  }
  return "?";
}

}  // namespace

LocalElevationMapperNode::LocalElevationMapperNode()
: rclcpp::Node("local_elevation_mapper_node")
{
  // --- parameters ---
  input_cloud_topic_       = declare_parameter<std::string>("input_cloud_topic", "/camera/camera/depth/color/points");
  accumulated_cloud_topic_ = declare_parameter<std::string>("accumulated_cloud_topic", "/local_elevation_map/accumulated_points");
  elevation_cloud_topic_   = declare_parameter<std::string>("elevation_cloud_topic", "/local_elevation_map/points");

  // target_frame is the gravity-aligned global frame the map lives in
  // (default `odom`, redefined by state_estimator to be world-up with
  // translation=[0,0,0] until real odometry is wired in).
  target_frame_      = declare_parameter<std::string>("target_frame", "odom");
  // track_point_frame is the body frame the local map window follows.
  // When ROI bounds are set, they are interpreted as offsets relative to
  // this frame's XY position in target_frame. For the stationary case
  // (translation=0) the configured bounds and the effective bounds are the
  // same; once translation is non-zero the grid auto-recenters.
  track_point_frame_ = declare_parameter<std::string>("track_point_frame", "base_link");
  k_frames_          = declare_parameter<int>("k_frames", 5);

  // map_length_x / map_length_y are kept in the yaml for documentation but the
  // grid is sized from x_max-x_min / y_max-y_min — declare them so loading the
  // yaml doesn't warn.
  declare_parameter<double>("map_length_x", 2.0);
  declare_parameter<double>("map_length_y", 2.0);

  resolution_ = declare_parameter<double>("resolution", 0.02);

  // ROI bounds: NaN default is a sentinel for "not provided in YAML".
  // When all six are finite, ROI crop is enabled and the grid extent is
  // fixed; when ANY is missing, we fall back to inspection mode (no crop,
  // grid auto-fits to the accumulated cloud's actual XY extent each frame).
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  x_min_ = declare_parameter<double>("x_min", kNaN);
  x_max_ = declare_parameter<double>("x_max", kNaN);
  y_min_ = declare_parameter<double>("y_min", kNaN);
  y_max_ = declare_parameter<double>("y_max", kNaN);
  z_min_ = declare_parameter<double>("z_min", kNaN);
  z_max_ = declare_parameter<double>("z_max", kNaN);

  roi_set_ =
    std::isfinite(x_min_) && std::isfinite(x_max_) &&
    std::isfinite(y_min_) && std::isfinite(y_max_) &&
    std::isfinite(z_min_) && std::isfinite(z_max_);

  publish_accumulated_cloud_ = declare_parameter<bool>("publish_accumulated_cloud", true);
  publish_elevation_cloud_   = declare_parameter<bool>("publish_elevation_cloud", true);

  min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 1);
  cloud_queue_size_    = declare_parameter<int>("cloud_queue_size", 5);
  tf_timeout_sec_      = declare_parameter<double>("tf_timeout_sec", 0.1);

  // When true, use the latest available TF instead of msg->header.stamp.
  // Useful when replaying a bag whose message stamps don't match /clock
  // (e.g. RealSense sensor hardware time vs bag record time).
  use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);

  // Per-cell fusion parameters (see FusionParams docs).
  fusion_params_.sensor_variance       = declare_parameter<double>("sensor_variance", 0.0009);
  fusion_params_.min_variance          = declare_parameter<double>("min_variance", 0.000009);
  fusion_params_.max_variance          = declare_parameter<double>("max_variance", 0.01);
  fusion_params_.mahalanobis_threshold = declare_parameter<double>("mahalanobis_threshold", 2.5);
  fusion_params_.multi_height_noise    = declare_parameter<double>("multi_height_noise", 0.0000009);
  enable_continuous_cleanup_           = declare_parameter<bool>("enable_continuous_cleanup", true);
  max_age_sec_                         = declare_parameter<double>("max_age_sec", 2.0);

  // --- Per-point measurement variance R (see FusionParams docs) ---
  const std::string mv_model_name =
    declare_parameter<std::string>("measurement_variance_model", "constant");
  const std::string depth_source_name =
    declare_parameter<std::string>("depth_source", "raw_z");
  fusion_params_.sensor_noise_factor =
    declare_parameter<double>("sensor_noise_factor", 0.0009);
  fusion_params_.min_measurement_variance =
    declare_parameter<double>("min_measurement_variance", 0.000009);
  fusion_params_.max_measurement_variance =
    declare_parameter<double>("max_measurement_variance", 0.01);
  fusion_params_.mahalanobis_use_measurement_variance =
    declare_parameter<bool>("mahalanobis_use_measurement_variance", false);
  debug_measurement_variance_ =
    declare_parameter<bool>("debug_measurement_variance", false);

  fusion_params_.measurement_variance_model = parse_mv_model(mv_model_name);
  fusion_params_.depth_source = parse_depth_source(depth_source_name);

  // --- TF ---
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // --- ROS ---
  rclcpp::QoS sensor_qos{rclcpp::KeepLast(static_cast<std::size_t>(cloud_queue_size_))};
  sensor_qos.best_effort();

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, sensor_qos,
    std::bind(&LocalElevationMapperNode::on_cloud, this, std::placeholders::_1));

  pub_accum_ = create_publisher<sensor_msgs::msg::PointCloud2>(accumulated_cloud_topic_, 1);
  pub_elev_  = create_publisher<sensor_msgs::msg::PointCloud2>(elevation_cloud_topic_, 1);

  if (roi_set_) {
    // Compute the initial grid spec just for the startup log. The runtime
    // fusion path builds its own frame_spec per callback (track-point
    // shifted in motion mode).
    const auto initial_spec =
      make_grid_spec(x_min_, x_max_, y_min_, y_max_, resolution_);
    RCLCPP_INFO(get_logger(),
      "Subscribed to %s -> target_frame=%s grid=%zux%zu@%.3fm k_frames=%d (ROI crop on)",
      input_cloud_topic_.c_str(), target_frame_.c_str(),
      initial_spec.size_x, initial_spec.size_y, resolution_, k_frames_);
  } else {
    RCLCPP_WARN(get_logger(),
      "Subscribed to %s -> target_frame=%s resolution=%.3fm k_frames=%d. "
      "INSPECTION MODE: not all of x_min/x_max/y_min/y_max/z_min/z_max are "
      "set in YAML, so ROI crop is disabled and the elevation grid extent "
      "auto-fits to the accumulated cloud each frame. Provide all six "
      "bounds in YAML for production.",
      input_cloud_topic_.c_str(), target_frame_.c_str(),
      resolution_, k_frames_);
  }

  RCLCPP_INFO(get_logger(),
    "Measurement variance model: %s, depth source: %s, sensor_variance=%.6g, "
    "sensor_noise_factor=%.6g, R clamp=[%.6g, %.6g], mahalanobis_use_R=%s",
    mv_model_str(fusion_params_.measurement_variance_model),
    depth_source_str(fusion_params_.depth_source),
    fusion_params_.sensor_variance,
    fusion_params_.sensor_noise_factor,
    fusion_params_.min_measurement_variance,
    fusion_params_.max_measurement_variance,
    fusion_params_.mahalanobis_use_measurement_variance ? "true" : "false");
}

void LocalElevationMapperNode::on_cloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // TF lookup time policy. (See Python `local_elevation_mapper_node.py` for
  // the long-form explanation.) In short:
  //   - use_latest_tf_ == true  -> tf2::TimePointZero ("latest available")
  //     This is the right default while running against rosbags whose
  //     message header.stamp is in sensor hardware time but /clock is in
  //     bag-record time (constant offset, can be seconds), or while the
  //     state estimator is the identity placeholder (TF doesn't vary in
  //     time anyway).
  //   - use_latest_tf_ == false -> exact lookup at msg->header.stamp.
  //     Switch to this once a real time-varying estimator is publishing
  //     odom->base_link AND cloud + TF share a time domain.
  const tf2::TimePoint lookup_time =
    use_latest_tf_
      ? tf2::TimePointZero
      : tf2_ros::fromMsg(msg->header.stamp);

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
      target_frame_,
      msg->header.frame_id,
      lookup_time,
      tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(),
      "TF lookup failed (%s -> %s): %s",
      msg->header.frame_id.c_str(), target_frame_.c_str(), e.what());
    return;
  }

  // Output stamp: latest mode -> "now" in node clock domain; exact mode ->
  // propagate the input cloud's stamp. Either choice keeps the output's
  // timestamps self-consistent for downstream consumers.
  const rclcpp::Time out_stamp =
    use_latest_tf_ ? this->now() : rclcpp::Time(msg->header.stamp);

  auto raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *raw);
  if (raw->empty()) {
    return;
  }

  // RealSense depth holes produce NaN points; drop them before any math.
  std::vector<int> nan_indices;
  pcl::removeNaNFromPointCloud(*raw, *raw, nan_indices);

  // Transform into the configured target frame using the TF we just looked up.
  const Eigen::Isometry3d transform_d = tf2::transformToEigen(tf_msg.transform);
  const Eigen::Affine3f transform_f(transform_d.cast<float>());
  auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*raw, *transformed, transform_f);

  // Look up the track point's XY in target_frame so the ROI / grid window
  // can follow the robot. With state_estimator forcing translation=0 this
  // is currently always (0,0,0) — but the lookup keeps the code ready for
  // when real odometry comes in.
  double tp_dx = 0.0, tp_dy = 0.0;
  if (track_point_frame_ != target_frame_) {
    try {
      const auto tp_tf = tf_buffer_->lookupTransform(
        target_frame_, track_point_frame_,
        lookup_time, tf2::durationFromSec(tf_timeout_sec_));
      tp_dx = tp_tf.transform.translation.x;
      tp_dy = tp_tf.transform.translation.y;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "track-point TF lookup failed (%s -> %s): %s. Treating offset as (0,0).",
        target_frame_.c_str(), track_point_frame_.c_str(), e.what());
    }
  }

  // ---------------------------------------------------------------------
  // Build per-point HeightMeasurements.
  //   raw[i] feeds R = compute_measurement_variance(...).
  //   transformed[i] gives target-frame xyz used for cell binning + fusion.
  //   removeNaN was done in-place on raw, then transformPointCloud preserves
  //   ordering, so raw[i] / transformed[i] always correspond.
  // ---------------------------------------------------------------------
  std::vector<HeightMeasurement> measurements;
  measurements.reserve(transformed->size());
  for (std::size_t i = 0; i < transformed->size(); ++i) {
    const auto & tp = transformed->points[i];
    const auto & rp = raw->points[i];
    measurements.push_back({
      tp.x, tp.y, tp.z,
      compute_measurement_variance(rp, fusion_params_),
    });
  }

  if (roi_set_) {
    crop_measurements_roi(measurements,
      static_cast<float>(x_min_ + tp_dx), static_cast<float>(x_max_ + tp_dx),
      static_cast<float>(y_min_ + tp_dy), static_cast<float>(y_max_ + tp_dy),
      static_cast<float>(z_min_),        static_cast<float>(z_max_));
  }

  // ---------------------------------------------------------------------
  // Pick this frame's grid spec.
  //   ROI mode      : configured bounds shifted by the track-point
  //                   translation (ego-centric in motion mode, identical
  //                   to configured bounds when translation = 0).
  //   Inspection    : auto-fit to this frame's measurement extent so every
  //                   valid point lands in some cell.
  // The bin stage uses this spec. The fusion stage will use a freshly
  // computed spec too — CellMeasurement stores world-frame cell centres,
  // so the two specs do not need to match (matters in inspection mode
  // where the auto-fit can drift frame to frame).
  // ---------------------------------------------------------------------
  GridSpec frame_spec;
  if (roi_set_) {
    frame_spec = make_grid_spec(
      x_min_ + tp_dx, x_max_ + tp_dx,
      y_min_ + tp_dy, y_max_ + tp_dy,
      resolution_);
  } else if (!measurements.empty()) {
    float min_x = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    for (const auto & m : measurements) {
      min_x = std::min(min_x, m.x);
      max_x = std::max(max_x, m.x);
      min_y = std::min(min_y, m.y);
      max_y = std::max(max_y, m.y);
    }
    frame_spec = make_grid_spec(
      static_cast<double>(min_x), static_cast<double>(max_x) + resolution_,
      static_cast<double>(min_y), static_cast<double>(max_y) + resolution_,
      resolution_);
  }

  // ---------------------------------------------------------------------
  // ❶ Bin this frame: raw measurements -> per-cell aggregates.
  //   Within-cell averaging finishes here in one O(N) pass: a cell that
  //   collected several raw points emerges as a single CellMeasurement
  //   carrying mean(z), mean(R), and the raw-point count.
  // ---------------------------------------------------------------------
  std::vector<CellMeasurement> binned_cells =
    bin_frame_into_cells(measurements, frame_spec);

  // ---------------------------------------------------------------------
  // ❷ Accumulate: push this frame's binned cells into the k_frames window.
  //   The buffer holds *cell-level* representations, not raw points. Memory
  //   stays proportional to occupied cells * k_frames, not point count *
  //   k_frames.
  // ---------------------------------------------------------------------
  measurement_buffer_.push_back({out_stamp.seconds(), std::move(binned_cells)});
  while (static_cast<int>(measurement_buffer_.size()) > k_frames_) {
    measurement_buffer_.pop_front();
  }

  // accumulated_points debug topic: cell-centre cloud of everything in the
  // window. One point per (frame, cell) pair (so the same physical cell
  // can produce up to k_frames points across the deque).
  if (publish_accumulated_cloud_) {
    pcl::PointCloud<pcl::PointXYZ> accum_cloud;
    std::size_t total = 0;
    for (const auto & f : measurement_buffer_) total += f.cells.size();
    accum_cloud.points.reserve(total);
    for (const auto & f : measurement_buffer_) {
      for (const auto & c : f.cells) {
        pcl::PointXYZ p;
        p.x = c.x; p.y = c.y; p.z = c.z;
        accum_cloud.points.push_back(p);
      }
    }
    accum_cloud.width = static_cast<std::uint32_t>(accum_cloud.points.size());
    accum_cloud.height = 1;
    accum_cloud.is_dense = true;
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(accum_cloud, out_msg);
    out_msg.header.stamp = out_stamp;
    out_msg.header.frame_id = target_frame_;
    pub_accum_->publish(std::move(out_msg));
  }

  // ---------------------------------------------------------------------
  // ❸ Fuse: temporal Kalman update of every buffered frame into layers_.
  //   continuous_cleanup=true (default for stationary use) wipes the
  //   layers at the start of each callback, so a 5-frame buffer means up
  //   to 5 sequential Kalman updates per cell — true temporal fusion.
  //
  //   Skip the whole stage when frame_spec is degenerate (size 0). That
  //   happens only in inspection mode + completely empty measurements;
  //   acting on a zero-size spec would silently wipe a persistent map
  //   (non-continuous_cleanup) via the shape_changed branch.
  // ---------------------------------------------------------------------
  if (publish_elevation_cloud_ && !measurement_buffer_.empty() &&
      frame_spec.size_x > 0 && frame_spec.size_y > 0) {
    const double stamp_sec = out_stamp.seconds();
    const bool shape_changed =
      layers_.height_map.size() != frame_spec.size_x * frame_spec.size_y;
    if (enable_continuous_cleanup_ || shape_changed) {
      reset_layers(frame_spec, layers_);
    } else {
      prune_stale_cells(layers_, stamp_sec, max_age_sec_);
    }

    for (const auto & f : measurement_buffer_) {
      fuse_binned_frame(f.cells, f.stamp_sec, frame_spec, fusion_params_, layers_);
    }

    if (debug_measurement_variance_) {
      double min_v = std::numeric_limits<double>::infinity();
      double max_v = -std::numeric_limits<double>::infinity();
      double sum_v = 0.0;
      std::size_t total = 0;
      for (const auto & f : measurement_buffer_) {
        for (const auto & c : f.cells) {
          if (c.variance < min_v) min_v = c.variance;
          if (c.variance > max_v) max_v = c.variance;
          sum_v += c.variance;
          ++total;
        }
      }
      if (total > 0) {
        const double mean_v = sum_v / static_cast<double>(total);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Cell-measurement variance stats: min=%.6g, mean=%.6g, max=%.6g, cells=%zu",
          min_v, mean_v, max_v, total);
      }
    }

    auto elev_pts = grid_to_xyzi_points(frame_spec, layers_, min_points_per_cell_);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*elev_pts, out_msg);
    out_msg.header.stamp = out_stamp;
    out_msg.header.frame_id = target_frame_;
    pub_elev_->publish(std::move(out_msg));
  }
}

}  // namespace realsense_elevation_mapper
