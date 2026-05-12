#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "realsense_elevation_mapper/local_elevation_mapper_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<realsense_elevation_mapper::LocalElevationMapperNode>());
  rclcpp::shutdown();
  return 0;
}
