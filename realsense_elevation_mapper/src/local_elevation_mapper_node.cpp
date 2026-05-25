#include "realsense_elevation_mapper/local_elevation_mapper_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
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
  fusion_params_.scanning_duration_sec = declare_parameter<double>("scanning_duration_sec", 0.5);
  enable_continuous_cleanup_           = declare_parameter<bool>("enable_continuous_cleanup", true);
  max_age_sec_                         = declare_parameter<double>("max_age_sec", 2.0);

  if (roi_set_) {
    grid_spec_ = make_grid_spec(x_min_, x_max_, y_min_, y_max_, resolution_);
  }

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
    RCLCPP_INFO(get_logger(),
      "Subscribed to %s -> target_frame=%s grid=%zux%zu@%.3fm k_frames=%d (ROI crop on)",
      input_cloud_topic_.c_str(), target_frame_.c_str(),
      grid_spec_.size_x, grid_spec_.size_y, resolution_, k_frames_);
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

  if (roi_set_) {
    crop_roi(*transformed,
             static_cast<float>(x_min_ + tp_dx), static_cast<float>(x_max_ + tp_dx),
             static_cast<float>(y_min_ + tp_dy), static_cast<float>(y_max_ + tp_dy),
             static_cast<float>(z_min_),        static_cast<float>(z_max_));
  }

  cloud_buffer_.push_back(transformed);
  while (static_cast<int>(cloud_buffer_.size()) > k_frames_) {
    cloud_buffer_.pop_front();
  }

  // Concatenate buffered clouds for both downstream outputs.
  auto accumulated = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::size_t total = 0;
  for (const auto & c : cloud_buffer_) {
    total += c->size();
  }
  accumulated->points.reserve(total);
  for (const auto & c : cloud_buffer_) {
    *accumulated += *c;
  }
  accumulated->width = static_cast<std::uint32_t>(accumulated->points.size());
  accumulated->height = 1;
  accumulated->is_dense = true;

  if (publish_accumulated_cloud_) {
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*accumulated, out_msg);
    out_msg.header.stamp = out_stamp;
    out_msg.header.frame_id = target_frame_;
    pub_accum_->publish(std::move(out_msg));
  }

  if (publish_elevation_cloud_ && !accumulated->empty()) {
    // Pick the grid spec for this frame. In ROI-set mode the bounds follow
    // the track point (configured bounds shifted by the target -> track
    // point translation). In inspection mode we auto-fit to the accumulated
    // cloud's actual XY extent so every valid point lands in some cell.
    // Adding one resolution to max guards against floating-point rounding
    // pushing the max-coordinate point one cell past the edge.
    GridSpec frame_spec;
    if (roi_set_) {
      frame_spec = make_grid_spec(
        x_min_ + tp_dx, x_max_ + tp_dx,
        y_min_ + tp_dy, y_max_ + tp_dy,
        resolution_);
    } else {
      float min_x = std::numeric_limits<float>::infinity();
      float max_x = -std::numeric_limits<float>::infinity();
      float min_y = std::numeric_limits<float>::infinity();
      float max_y = -std::numeric_limits<float>::infinity();
      for (const auto & p : accumulated->points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
      }
      frame_spec = make_grid_spec(
        static_cast<double>(min_x), static_cast<double>(max_x) + resolution_,
        static_cast<double>(min_y), static_cast<double>(max_y) + resolution_,
        resolution_);
    }

    const double stamp_sec = out_stamp.seconds();

    // Continuous cleanup mode (default for the stationary case): start with
    // a fresh grid every callback so the map mirrors the current
    // accumulated buffer only. Otherwise reuse the persistent grid; if its
    // shape changed we still have to resize, and stale cells are aged out
    // by max_age_sec.
    const bool shape_changed =
      layers_.height_map.size() != frame_spec.size_x * frame_spec.size_y;
    if (enable_continuous_cleanup_ || shape_changed) {
      reset_layers(frame_spec, layers_);
    } else {
      prune_stale_cells(layers_, stamp_sec, max_age_sec_);
    }

    fuse_cloud(*accumulated, stamp_sec, frame_spec, fusion_params_, layers_);
    auto elev_pts = grid_to_xyzi_points(frame_spec, layers_, min_points_per_cell_);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*elev_pts, out_msg);
    out_msg.header.stamp = out_stamp;
    out_msg.header.frame_id = target_frame_;
    pub_elev_->publish(std::move(out_msg));
  }
}

}  // namespace realsense_elevation_mapper
