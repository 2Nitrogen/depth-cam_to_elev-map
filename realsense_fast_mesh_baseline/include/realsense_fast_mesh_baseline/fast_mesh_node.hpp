// FastMeshNode — single-frame vanilla baseline.
//
// Subscribes to a depth Image + its CameraInfo (NOT to a PointCloud2),
// and reconstructs the organized point cloud itself. This sidesteps the
// realsense2_camera wrapper's `pointcloud.ordered_pc` flag — many bags
// were recorded with that flag false, producing a NaN-stripped /
// unorganized PointCloud2 topic that the PCL organized algorithms
// cannot consume. Going through depth image + CameraInfo gives us a
// guaranteed organized cloud in any environment.
//
// Per-callback flow (driven by the depth-image subscription; CameraInfo
// is cached on its own callback so the depth callback always uses the
// latest available intrinsics):
//
//   1. ingest depth Image + cached CameraInfo, build organized
//      PointCloud<PointXYZ> in camera (optical) frame.
//   2. estimate per-pixel normals in camera frame
//      (IntegralImageNormalEstimation needs z = depth axis).
//   3. TF lookup target ← cam (frame from depth image header).
//   4. transform cloud + rotate normals into target_frame.
//   5. OrganizedFastMesh → triangle mesh in target frame.
//   6. serialize to visualization_msgs/Marker (TRIANGLE_LIST) with
//      per-vertex slope coloring.

#ifndef REALSENSE_FAST_MESH_BASELINE__FAST_MESH_NODE_HPP_
#define REALSENSE_FAST_MESH_BASELINE__FAST_MESH_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "realsense_fast_mesh_baseline/mesh_builder.hpp"
#include "realsense_fast_mesh_baseline/mesh_marker.hpp"

namespace realsense_fast_mesh_baseline
{

class FastMeshNode : public rclcpp::Node
{
public:
  FastMeshNode();

private:
  void on_depth_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  // ---- topics / frames ----
  std::string depth_image_topic_;
  std::string camera_info_topic_;
  std::string mesh_marker_topic_;
  std::string target_frame_;

  // ---- timing / TF ----
  double tf_timeout_sec_{0.1};
  bool   use_latest_tf_{true};
  int    cloud_queue_size_{5};

  // ---- algorithmic / visualization params ----
  MeshBuilderParams builder_params_{};
  MarkerStyle       marker_style_{};

  // ---- runtime state ----
  // CameraInfo is cached on its own callback so the depth-image
  // callback always uses the latest available intrinsics. RealSense
  // ROS2 wrapper publishes CameraInfo at the same rate as the depth
  // image so the cache is essentially always current.
  sensor_msgs::msg::CameraInfo::ConstSharedPtr cached_camera_info_;

  // ---- TF ----
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- ROS interfaces ----
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
};

}  // namespace realsense_fast_mesh_baseline

#endif  // REALSENSE_FAST_MESH_BASELINE__FAST_MESH_NODE_HPP_
