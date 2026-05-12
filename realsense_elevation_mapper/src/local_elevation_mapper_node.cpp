#include "realsense_elevation_mapper/local_elevation_mapper_node.hpp"

#include <chrono>
#include <functional>
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

  target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
  k_frames_     = declare_parameter<int>("k_frames", 5);

  // map_length_x / map_length_y are kept in the yaml for documentation but the
  // grid is sized from x_max-x_min / y_max-y_min — declare them so loading the
  // yaml doesn't warn.
  declare_parameter<double>("map_length_x", 2.0);
  declare_parameter<double>("map_length_y", 2.0);

  resolution_ = declare_parameter<double>("resolution", 0.02);
  x_min_      = declare_parameter<double>("x_min", 0.0);
  x_max_      = declare_parameter<double>("x_max", 2.0);
  y_min_      = declare_parameter<double>("y_min", -1.0);
  y_max_      = declare_parameter<double>("y_max", 1.0);
  z_min_      = declare_parameter<double>("z_min", -0.5);
  z_max_      = declare_parameter<double>("z_max", 1.0);

  // When false, ROI cropping is skipped: every transformed point goes
  // into the accumulated buffer regardless of x_min/x_max/.. bounds.
  // Useful during bring-up to inspect the raw scene in RViz and decide
  // appropriate crop bounds. The elevation grid still uses the configured
  // map_length / resolution and silently drops points outside it — widen
  // those if you want a full-coverage elevation cloud too. Re-enable for
  // production: cropping is what keeps the accumulated buffer small enough
  // to grid-bin every callback.
  enable_roi_crop_ = declare_parameter<bool>("enable_roi_crop", true);

  publish_accumulated_cloud_ = declare_parameter<bool>("publish_accumulated_cloud", true);
  publish_elevation_cloud_   = declare_parameter<bool>("publish_elevation_cloud", true);

  min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 1);
  cloud_queue_size_    = declare_parameter<int>("cloud_queue_size", 5);
  tf_timeout_sec_      = declare_parameter<double>("tf_timeout_sec", 0.1);

  // When true, use the latest available TF instead of msg->header.stamp.
  // Useful when replaying a bag whose message stamps don't match /clock
  // (e.g. RealSense sensor hardware time vs bag record time).
  use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);

  grid_spec_ = make_grid_spec(x_min_, x_max_, y_min_, y_max_, resolution_);

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

  RCLCPP_INFO(get_logger(),
    "Subscribed to %s -> target_frame=%s grid=%zux%zu@%.3fm k_frames=%d",
    input_cloud_topic_.c_str(), target_frame_.c_str(),
    grid_spec_.size_x, grid_spec_.size_y, resolution_, k_frames_);
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

  if (enable_roi_crop_) {
    crop_roi(*transformed,
             static_cast<float>(x_min_), static_cast<float>(x_max_),
             static_cast<float>(y_min_), static_cast<float>(y_max_),
             static_cast<float>(z_min_), static_cast<float>(z_max_));
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

  if (publish_elevation_cloud_) {
    std::vector<float> height_map;
    std::vector<int> count_map;
    compute_mean_elevation(*accumulated, grid_spec_, height_map, count_map);
    auto elev_pts = grid_to_points(height_map, count_map, grid_spec_, min_points_per_cell_);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*elev_pts, out_msg);
    out_msg.header.stamp = out_stamp;
    out_msg.header.frame_id = target_frame_;
    pub_elev_->publish(std::move(out_msg));
  }
}

}  // namespace realsense_elevation_mapper
