#include "realsense_fast_mesh_baseline/fast_mesh_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace realsense_fast_mesh_baseline
{

FastMeshNode::FastMeshNode()
: rclcpp::Node("local_fast_mesh_node")
{
  // ---- parameters ----
  depth_image_topic_ = declare_parameter<std::string>(
    "depth_image_topic", "/camera/camera/depth/image_rect_raw");
  camera_info_topic_ = declare_parameter<std::string>(
    "camera_info_topic", "/camera/camera/depth/camera_info");
  accumulated_cloud_topic_ = declare_parameter<std::string>(
    "accumulated_cloud_topic", "/local_fast_mesh/accumulated_cloud");
  target_frame_ = declare_parameter<std::string>("target_frame", "odom");

  tf_timeout_sec_   = declare_parameter<double>("tf_timeout_sec",   0.1);
  use_latest_tf_    = declare_parameter<bool>(  "use_latest_tf",    true);
  cloud_queue_size_ = declare_parameter<int>(   "cloud_queue_size", 5);

  {
    const int s = declare_parameter<int>("pixel_stride", 1);
    builder_params_.pixel_stride = static_cast<std::uint32_t>(std::max(1, s));
  }
  builder_params_.max_distance_m =
    static_cast<float>(declare_parameter<double>("max_distance_m", 3.5));

  {
    const int k = declare_parameter<int>("buffer_size_k", 10);
    buffer_size_k_ = static_cast<std::uint32_t>(std::max(1, k));
  }

  // ---- TF ----
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- ROS ----
  rclcpp::QoS sensor_qos{rclcpp::KeepLast(
    static_cast<std::size_t>(cloud_queue_size_))};
  sensor_qos.best_effort();

  sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
    depth_image_topic_, sensor_qos,
    std::bind(&FastMeshNode::on_depth_image, this, std::placeholders::_1));
  sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::QoS{1}.reliable(),
    std::bind(&FastMeshNode::on_camera_info, this, std::placeholders::_1));
  pub_cloud_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(accumulated_cloud_topic_, 1);

  RCLCPP_INFO(get_logger(),
    "Subscribed to depth=%s + camera_info=%s -> target_frame=%s "
    "pixel_stride=%u max_distance=%.2fm buffer_K=%u "
    "publishing accumulated cloud on %s",
    depth_image_topic_.c_str(), camera_info_topic_.c_str(),
    target_frame_.c_str(),
    builder_params_.pixel_stride,
    builder_params_.max_distance_m,
    buffer_size_k_,
    accumulated_cloud_topic_.c_str());
}

void FastMeshNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  cached_camera_info_ = msg;
}

void FastMeshNode::on_depth_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!cached_camera_info_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Depth image arrived but no CameraInfo cached yet on %s. Skipping.",
      camera_info_topic_.c_str());
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(),
    "[1/4] First depth callback: encoding=%s %ux%u",
    msg->encoding.c_str(), msg->width, msg->height);

  // ---- Build organized cloud from depth image + intrinsics ----
  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud;
  try {
    cam_cloud = build_organized_cloud_from_depth(*msg, *cached_camera_info_, builder_params_);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "Failed to build cloud from depth image: %s", e.what());
    return;
  }
  if (cam_cloud->empty()) {
    return;
  }
  RCLCPP_INFO_ONCE(get_logger(),
    "[2/4] Cloud built: %ux%u (%zu pts)",
    cam_cloud->width, cam_cloud->height, cam_cloud->points.size());

  // ---- TF lookup ----
  const tf2::TimePoint lookup_time =
    use_latest_tf_
      ? tf2::TimePointZero
      : tf2_ros::fromMsg(msg->header.stamp);

  geometry_msgs::msg::TransformStamped tf_cam_to_target;
  try {
    tf_cam_to_target = tf_buffer_->lookupTransform(
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
  const rclcpp::Time out_stamp =
    use_latest_tf_ ? this->now() : rclcpp::Time(msg->header.stamp);

  // ---- Transform cloud into target frame ----
  const Eigen::Isometry3d transform_d = tf2::transformToEigen(tf_cam_to_target.transform);
  const Eigen::Matrix4f T4 = transform_d.matrix().cast<float>();
  const Eigen::Affine3f transform_f(T4);
  auto tgt_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*cam_cloud, *tgt_cloud, transform_f);
  RCLCPP_INFO_ONCE(get_logger(),
    "[3/4] Cloud transformed to %s", target_frame_.c_str());

  // ---- Push to K-frame FIFO buffer ----
  cloud_buffer_.push_back(tgt_cloud);
  while (cloud_buffer_.size() > buffer_size_k_) {
    cloud_buffer_.pop_front();
  }

  // ---- Concatenate buffered clouds, drop NaN, publish ----
  pcl::PointCloud<pcl::PointXYZ> accumulated;
  std::size_t reserve_total = 0;
  for (const auto & c : cloud_buffer_) {
    reserve_total += c->points.size();
  }
  accumulated.points.reserve(reserve_total);
  for (const auto & c : cloud_buffer_) {
    for (const auto & pt : c->points) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        accumulated.points.push_back(pt);
      }
    }
  }
  accumulated.width = static_cast<std::uint32_t>(accumulated.points.size());
  accumulated.height = 1u;
  accumulated.is_dense = true;

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(accumulated, out_msg);
  out_msg.header.stamp = out_stamp;
  out_msg.header.frame_id = target_frame_;
  const std::size_t out_points = accumulated.points.size();
  pub_cloud_->publish(std::move(out_msg));
  RCLCPP_INFO_ONCE(get_logger(),
    "[4/4] First accumulated cloud published (buf=%zu, points=%zu)",
    cloud_buffer_.size(), out_points);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "Published: buf=%zu  points=%zu",
    cloud_buffer_.size(), out_points);
}

}  // namespace realsense_fast_mesh_baseline
