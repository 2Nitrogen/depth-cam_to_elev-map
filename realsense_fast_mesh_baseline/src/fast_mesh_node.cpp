#include "realsense_fast_mesh_baseline/fast_mesh_node.hpp"

#include <chrono>
#include <functional>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>

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
  mesh_marker_topic_ = declare_parameter<std::string>(
    "mesh_marker_topic", "/local_fast_mesh/mesh");
  target_frame_      = declare_parameter<std::string>("target_frame", "odom");

  tf_timeout_sec_   = declare_parameter<double>("tf_timeout_sec",   0.1);
  use_latest_tf_    = declare_parameter<bool>(  "use_latest_tf",    true);
  cloud_queue_size_ = declare_parameter<int>(   "cloud_queue_size", 5);

  // pixel_stride: ROS2 parameter API doesn't have uint32, so accept as
  // int (default 1) and clamp to >=1 before stashing.
  {
    const int s = declare_parameter<int>("pixel_stride", 1);
    builder_params_.pixel_stride = static_cast<std::uint32_t>(std::max(1, s));
  }
  builder_params_.normal_smoothing_size =
    static_cast<float>(declare_parameter<double>("normal_smoothing_size", 20.0));
  builder_params_.max_depth_change_factor =
    static_cast<float>(declare_parameter<double>("max_depth_change_factor", 0.02));
  // triangle_max_edge_length is intentionally NOT a yaml knob — it
  // scales linearly with pixel_stride via kBaseEdgeLengthPerStride so
  // that the rejected-as-discontinuity threshold tracks vertex spacing
  // automatically. See mesh_builder.hpp for the rationale.
  builder_params_.triangle_max_edge_length =
    static_cast<float>(builder_params_.pixel_stride) * kBaseEdgeLengthPerStride;
  builder_params_.triangulation_type = parse_triangulation_type(
    declare_parameter<std::string>("triangulation_type", "TRIANGLE_ADAPTIVE_CUT"));

  marker_style_.color_min_slope_rad =
    declare_parameter<double>("color_min_slope_rad", 0.0);
  marker_style_.color_max_slope_rad =
    declare_parameter<double>("color_max_slope_rad", 0.785);
  marker_style_.point_size_m =
    declare_parameter<double>("point_size_m", 0.01);
  marker_style_.face_alpha =
    declare_parameter<double>("face_alpha", 0.3);
  marker_style_.edge_width_m =
    declare_parameter<double>("edge_width_m", 0.002);
  marker_style_.edge_alpha =
    declare_parameter<double>("edge_alpha", 0.5);

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
  // CameraInfo is published with a reliability QoS on the wrapper side;
  // use a small KEEP_LAST + reliable subscription. We only need the
  // latest value, so depth=1 is enough.
  sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::QoS{1}.reliable(),
    std::bind(&FastMeshNode::on_camera_info, this, std::placeholders::_1));
  pub_marker_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(mesh_marker_topic_, 1);

  RCLCPP_INFO(get_logger(),
    "Subscribed to depth=%s + camera_info=%s -> target_frame=%s "
    "pixel_stride=%u normal_smoothing=%.2fpx max_depth_change=%.4f "
    "triangle_max_edge=%.3fm triangulation=%s color_slope=[%.3f..%.3f]rad "
    "point_size=%.3fm face_alpha=%.2f edge_width=%.4fm edge_alpha=%.2f",
    depth_image_topic_.c_str(), camera_info_topic_.c_str(),
    target_frame_.c_str(),
    builder_params_.pixel_stride,
    builder_params_.normal_smoothing_size,
    builder_params_.max_depth_change_factor,
    builder_params_.triangle_max_edge_length,
    triangulation_type_str(builder_params_.triangulation_type),
    marker_style_.color_min_slope_rad,
    marker_style_.color_max_slope_rad,
    marker_style_.point_size_m,
    marker_style_.face_alpha,
    marker_style_.edge_width_m,
    marker_style_.edge_alpha);
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
    "[1/7] First depth callback: encoding=%s %ux%u",
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
    "[2/7] Cloud built: %ux%u (%zu pts) organized=%d is_dense=%d",
    cam_cloud->width, cam_cloud->height, cam_cloud->points.size(),
    static_cast<int>(cam_cloud->isOrganized()),
    static_cast<int>(cam_cloud->is_dense));

  // ---- Estimate normals on CAMERA-frame cloud (z = optical depth) ----
  auto cam_normals = estimate_normals(cam_cloud, builder_params_);
  RCLCPP_INFO_ONCE(get_logger(),
    "[3/7] Normals estimated: %zu pts (organized=%d)",
    cam_normals->points.size(),
    static_cast<int>(cam_normals->isOrganized()));

  // ---- TF lookup (mirror surfel/elev mapper) ----
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
  RCLCPP_INFO_ONCE(get_logger(), "[4/7] TF lookup ok");

  // ---- Transform cloud into target frame (preserves organized) ----
  // Build the Affine3f via an explicit Matrix4f cast (avoids the
  // Isometry3f-to-Affine3f constructor path that has been observed to
  // misbehave on some PCL 1.12 / Eigen 3.4 combinations).
  const Eigen::Isometry3d transform_d = tf2::transformToEigen(tf_cam_to_target.transform);
  const Eigen::Matrix4f T4 = transform_d.matrix().cast<float>();
  const Eigen::Affine3f transform_f(T4);
  auto tgt_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*cam_cloud, *tgt_cloud, transform_f);
  RCLCPP_INFO_ONCE(get_logger(),
    "[5/7] Cloud transformed: %ux%u organized=%d",
    tgt_cloud->width, tgt_cloud->height,
    static_cast<int>(tgt_cloud->isOrganized()));

  // ---- Rotate normals into target frame (rotation only) ----
  // Use Matrix4f top-left block for the rotation — same rationale as
  // above, avoids the transform_f.linear() Block-to-Matrix3f path.
  const Eigen::Matrix3f R = T4.topLeftCorner<3, 3>();
  auto tgt_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  tgt_normals->width  = cam_normals->width;
  tgt_normals->height = cam_normals->height;
  tgt_normals->is_dense = cam_normals->is_dense;
  tgt_normals->points.resize(cam_normals->points.size());
  for (std::size_t i = 0; i < cam_normals->points.size(); ++i) {
    const auto & nc = cam_normals->points[i];
    Eigen::Vector3f n_in(nc.normal_x, nc.normal_y, nc.normal_z);
    const Eigen::Vector3f n_out = R * n_in;
    auto & no = tgt_normals->points[i];
    no.normal_x = n_out.x();
    no.normal_y = n_out.y();
    no.normal_z = n_out.z();
    no.curvature = nc.curvature;
  }
  RCLCPP_INFO_ONCE(get_logger(), "[6/7] Normals rotated");

  // ---- Build mesh on the target-frame cloud ----
  pcl::PolygonMesh mesh = build_fast_mesh(tgt_cloud, builder_params_);
  RCLCPP_INFO_ONCE(get_logger(),
    "[7/7] Mesh built: %zu polygons", mesh.polygons.size());

  // ---- Serialize to MarkerArray + publish ----
  auto marker_array = mesh_to_marker_array(
    mesh, *tgt_normals, out_stamp, target_frame_, marker_style_);
  pub_marker_->publish(std::move(marker_array));
}

}  // namespace realsense_fast_mesh_baseline
