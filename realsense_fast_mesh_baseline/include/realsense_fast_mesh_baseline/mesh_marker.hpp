// Convert a pcl::PolygonMesh + per-face normals into a
// visualization_msgs/MarkerArray containing three markers:
//
//   id=0  POINTS         — opaque vertex dots. Per-vertex color is the
//                          slope of the AVERAGE incident face normal
//                          (Gouraud-style — each vertex aggregates the
//                          face_normals of the triangles using it).
//   id=1  TRIANGLE_LIST  — faint triangle faces (slope-colored from
//                          face_normals, flat shading: all 3 vertices
//                          of a triangle share the face's color).
//                          Alpha = face_alpha.
//   id=2  LINE_LIST      — wireframe edges (uniform light gray, alpha
//                          = edge_alpha, width = edge_width_m).
//
// Combined effect in RViz: vertex dots show smooth gradient across
// curved regions, faces show flat per-triangle slope (cleanly stepped
// at orientation changes), edges convey mesh connectivity. All three
// markers use action=ADD with fixed ids so consecutive publishes
// overwrite the previous instances — no DELETE bookkeeping needed.

#ifndef REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_
#define REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>

#include <pcl/PolygonMesh.h>

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
// pcl::PointXYZ in `frame_id` coordinates. `face_normals` must be 1:1
// with mesh.polygons (same length, same order) — typically produced by
// `compute_face_normals(mesh)` — and expressed in the SAME frame as
// the mesh vertices.
visualization_msgs::msg::MarkerArray mesh_to_marker_array(
  const pcl::PolygonMesh & mesh,
  const std::vector<Eigen::Vector3f> & face_normals,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const MarkerStyle & style);

}  // namespace realsense_fast_mesh_baseline

#endif  // REALSENSE_FAST_MESH_BASELINE__MESH_MARKER_HPP_
