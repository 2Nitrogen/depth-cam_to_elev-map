#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "realsense_fast_mesh_baseline/fast_mesh_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<realsense_fast_mesh_baseline::FastMeshNode>());
  rclcpp::shutdown();
  return 0;
}
