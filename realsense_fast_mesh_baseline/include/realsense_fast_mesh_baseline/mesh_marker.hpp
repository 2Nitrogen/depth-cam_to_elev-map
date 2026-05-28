// Convert a pcl::PolygonMesh + per-vertex normals into one
// visualization_msgs/Marker (TRIANGLE_LIST), with each vertex's color
// encoding the slope angle of its local normal relative to +z (= world
// up, since publishing in the gravity-aligned target frame).
//
// The Marker uses id=0 with action=ADD; consecutive publishes simply
// overwrite the previous instance in RViz, so no DELETE bookkeeping
// is needed.

#ifndef REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_
#define REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_

#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace realsense_fast_mesh_baseline
{

struct MarkerStyle
{
  // Slope angle (rad) at which the green→red ramp starts (green) and
  // ends (red). Slope angle of a triangle's vertex normal is computed
  // as arccos(|n_z|), assuming the marker frame is gravity-aligned.
  double color_min_slope_rad{0.0};
  double color_max_slope_rad{0.785};
};

// Build the Marker. `mesh.cloud` must be a sensor_msgs/PointCloud2 of
// pcl::PointXYZ in `frame_id` coordinates; `vertex_normals` must be
// aligned 1:1 to that cloud (same width × height, same index order)
// and expressed in the SAME frame as the cloud.
visualization_msgs::msg::Marker mesh_to_triangle_list_marker(
  const pcl::PolygonMesh & mesh,
  const pcl::PointCloud<pcl::Normal> & vertex_normals,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const MarkerStyle & style);

}  // namespace realsense_fast_mesh_baseline

#endif  // REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_
