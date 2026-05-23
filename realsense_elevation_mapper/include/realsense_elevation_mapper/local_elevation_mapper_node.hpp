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

// Per-callback data pipeline shapes:
//
//   in   : sensor_msgs/PointCloud2     N points, fields x,y,z (float32, m)
//                                      + rgb (float32 packed), frame =
//                                      camera_*_optical_frame, NaN allowed.
//   raw  : pcl::PointCloud<PointXYZ>   M <= N (NaN removed), optical frame.
//   tf'd : pcl::PointCloud<PointXYZ>   M, frame = target_frame (base_link).
//   crop : same M' <= M, applied only when all six ROI bounds are
//          provided in YAML (otherwise inspection mode -> no crop).
//   buf  : deque of up to k_frames clouds; concat -> accumulated of
//          size sum_i |c_i|.
//   grid : std::vector<float> height_map, size = size_x * size_y,
//          row-major with idx = ix * size_y + iy, NaN where count == 0.
//          std::vector<int>   count_map  (parallel).
//   out_a: sensor_msgs/PointCloud2 sum_i |c_i| points (x,y,z float32),
//          frame = target_frame.
//   out_e: sensor_msgs/PointCloud2 <= size_x * size_y points (one per
//          occupied cell), (x,y) at cell center, z = per-cell estimate.
//
// ROI / grid mode (chosen once at construction):
//   * roi_set_ == true : ROI crop active, grid_spec_ static from YAML.
//   * roi_set_ == false: inspection mode, no crop, grid_spec rebuilt
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
  std::string target_frame_;
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

  // derived state
  GridSpec grid_spec_;
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_buffer_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_accum_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elev_;
};

}  // namespace realsense_elevation_mapper
