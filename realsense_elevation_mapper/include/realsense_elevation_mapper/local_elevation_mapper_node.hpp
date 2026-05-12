#pragma once

#include <deque>
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

  double x_min_{0.0};
  double x_max_{2.0};
  double y_min_{-1.0};
  double y_max_{1.0};
  double z_min_{-0.5};
  double z_max_{1.0};
  double resolution_{0.02};

  bool publish_accumulated_cloud_{true};
  bool publish_elevation_cloud_{true};
  bool enable_roi_crop_{true};
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
