// Convert a pcl::PolygonMesh + per-vertex normals into a
// visualization_msgs/MarkerArray containing three markers:
//
//   id=0  POINTS         — opaque vertex dots (slope-colored).
//   id=1  TRIANGLE_LIST  — faint triangle faces (slope-colored,
//                          alpha = face_alpha).
//   id=2  LINE_LIST      — wireframe edges (uniform light gray,
//                          alpha = edge_alpha, width = edge_width_m).
//
// Combined effect in RViz: vertex dots clearly visible, edges drawn
// in pale gray to convey mesh connectivity, face surfaces softly
// shaded behind. All three markers use action=ADD with fixed ids so
// consecutive publishes overwrite the previous instances — no DELETE
// bookkeeping needed.

#ifndef REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_
#define REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_

#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace realsense_fast_mesh_baseline
{

struct MarkerStyle
{
  // Slope angle (rad) at which the green→red ramp starts (green) and
  // ends (red).
  double color_min_slope_rad{0.0};
  double color_max_slope_rad{0.785};

  // Vertex (POINTS marker) point size in meters. Set to the typical
  // 3D inter-vertex spacing at the working depth for non-overlapping
  // dots; a few mm larger if you want them to bleed together visually.
  double point_size_m{0.01};

  // Triangle face (TRIANGLE_LIST marker) alpha in [0, 1]. Lower = more
  // see-through, lets the vertex POINTS dominate the visualization.
  double face_alpha{0.3};

  // Wireframe (LINE_LIST marker) line width in meters and alpha. The
  // color is hardcoded to a neutral light gray (0.7, 0.7, 0.7) so that
  // edges visually subordinate themselves to the slope-colored faces
  // and vertices.
  double edge_width_m{0.002};
  double edge_alpha{0.5};
};

// Build the MarkerArray. `mesh.cloud` must be a pcl::PCLPointCloud2 of
// pcl::PointXYZ in `frame_id` coordinates; `vertex_normals` must be
// aligned 1:1 to that cloud (same index order) and expressed in the
// SAME frame as the cloud.
visualization_msgs::msg::MarkerArray mesh_to_marker_array(
  const pcl::PolygonMesh & mesh,
  const pcl::PointCloud<pcl::Normal> & vertex_normals,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const MarkerStyle & style);

}  // namespace realsense_fast_mesh_baseline

#endif  // REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_
