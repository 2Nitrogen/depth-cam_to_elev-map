// FastMeshNode — K-frame raw point cloud accumulator.
//
// Subscribes to a depth Image + its CameraInfo, builds an organized
// PointCloud<XYZ> per frame, transforms it into the mapping
// (gravity-aligned) frame, pushes it into a fixed-size FIFO buffer,
// and publishes the union of all buffered frames as a single
// sensor_msgs/PointCloud2 every callback.
//
// No mesh, no normals, no fusion: this is the raw substrate over which
// downstream consumers (e.g. a Gaussian-boundary correspondence + mesh
// extractor) will operate. Same physical point will appear K times if
// observed from K different frames — that's the input the downstream
// algorithm is designed to consume.
//
// Per-callback flow:
//   1. ingest depth Image + cached CameraInfo, build organized
//      PointCloud<XYZ> in the camera optical frame.
//   2. TF lookup target ← cam.
//   3. transform cloud into target_frame.
//   4. push to K-frame FIFO buffer (oldest popped if size > K).
//   5. concat all buffered clouds (NaN dropped) → publish PointCloud2
//      with header.frame_id = target_frame.

#ifndef REALSENSE_FAST_MESH_BASELINE__FAST_MESH_NODE_HPP_
#define REALSENSE_FAST_MESH_BASELINE__FAST_MESH_NODE_HPP_

#include <deque>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "realsense_fast_mesh_baseline/mesh_builder.hpp"

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
  std::string accumulated_cloud_topic_;
  std::string target_frame_;

  // ---- timing / TF ----
  double tf_timeout_sec_{0.1};
  bool   use_latest_tf_{true};
  int    cloud_queue_size_{5};

  // ---- cloud build ----
  MeshBuilderParams builder_params_{};

  // ---- K-frame accumulation ----
  std::uint32_t buffer_size_k_{10u};
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_buffer_;

  // ---- runtime state ----
  sensor_msgs::msg::CameraInfo::ConstSharedPtr cached_camera_info_;

  // ---- TF ----
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- ROS interfaces ----
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
};

}  // namespace realsense_fast_mesh_baseline

#endif  // REALSENSE_FAST_MESH_BASELINE__FAST_MESH_NODE_HPP_
