#include "realsense_fast_mesh_baseline/mesh_marker.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/msg/marker.hpp>

namespace realsense_fast_mesh_baseline
{
namespace
{

std_msgs::msg::ColorRGBA slope_color(double t, double alpha)
{
  t = std::max(0.0, std::min(1.0, t));
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(t);
  c.g = static_cast<float>(1.0 - t);
  c.b = 0.0f;
  c.a = static_cast<float>(std::max(0.0, std::min(1.0, alpha)));
  return c;
}

// Slope (rad) of a vertex's normal relative to +z. NaN-safe: returns
// 0 (flat) for degenerate / missing normals.
double slope_from_normal(const pcl::Normal & n)
{
  const float nx = n.normal_x;
  const float ny = n.normal_y;
  const float nz = n.normal_z;
  if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(nz)) {
    return 0.0;
  }
  const double mag = std::sqrt(
    static_cast<double>(nx) * nx +
    static_cast<double>(ny) * ny +
    static_cast<double>(nz) * nz);
  if (mag <= 0.0) {
    return 0.0;
  }
  const double cos_abs = std::min(1.0, std::abs(static_cast<double>(nz) / mag));
  return std::acos(cos_abs);
}

double slope_to_ramp_t(
  const pcl::Normal & n,
  const MarkerStyle & style,
  double slope_range)
{
  const double slope = slope_from_normal(n);
  return (slope - style.color_min_slope_rad) / slope_range;
}

}  // namespace

visualization_msgs::msg::MarkerArray mesh_to_marker_array(
  const pcl::PolygonMesh & mesh,
  const pcl::PointCloud<pcl::Normal> & vertex_normals,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const MarkerStyle & style)
{
  visualization_msgs::msg::MarkerArray arr;

  // Extract vertices from PCL's serialized cloud format into an
  // in-memory PointCloud<PointXYZ> for index-based access.
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  const std::size_t n_vertices = vertices.points.size();
  const std::size_t n_normals  = vertex_normals.points.size();
  const bool have_normals = (n_normals == n_vertices);

  const double slope_range =
    std::max(1e-6, style.color_max_slope_rad - style.color_min_slope_rad);

  // ---- Marker id=0: POINTS (vertices, opaque) -----------------------
  visualization_msgs::msg::Marker pts;
  pts.header.stamp = stamp;
  pts.header.frame_id = frame_id;
  pts.ns = "fast_mesh";
  pts.id = 0;
  pts.type = visualization_msgs::msg::Marker::POINTS;
  pts.action = visualization_msgs::msg::Marker::ADD;
  pts.pose.orientation.w = 1.0;
  // For POINTS: scale.x = width (m), scale.y = height (m) in world.
  pts.scale.x = style.point_size_m;
  pts.scale.y = style.point_size_m;

  pts.points.reserve(n_vertices);
  pts.colors.reserve(n_vertices);

  for (std::size_t i = 0; i < n_vertices; ++i) {
    const auto & v = vertices.points[i];
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
      continue;
    }
    geometry_msgs::msg::Point p;
    p.x = v.x; p.y = v.y; p.z = v.z;
    pts.points.push_back(p);

    double t = 0.0;
    if (have_normals) {
      t = slope_to_ramp_t(vertex_normals.points[i], style, slope_range);
    }
    pts.colors.push_back(slope_color(t, 1.0));  // opaque
  }
  arr.markers.push_back(std::move(pts));

  // ---- Marker id=1: TRIANGLE_LIST (faces, faint) --------------------
  visualization_msgs::msg::Marker tris;
  tris.header.stamp = stamp;
  tris.header.frame_id = frame_id;
  tris.ns = "fast_mesh";
  tris.id = 1;
  tris.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  tris.action = visualization_msgs::msg::Marker::ADD;
  tris.pose.orientation.w = 1.0;
  // TRIANGLE_LIST: scale is unused but must be non-zero to avoid
  // rendering as a degenerate marker in some RViz versions.
  tris.scale.x = 1.0;
  tris.scale.y = 1.0;
  tris.scale.z = 1.0;

  tris.points.reserve(mesh.polygons.size() * 3);
  tris.colors.reserve(mesh.polygons.size() * 3);

  for (const auto & poly : mesh.polygons) {
    if (poly.vertices.size() != 3) {
      continue;  // OrganizedFastMesh always emits triangles; defensive.
    }
    for (std::size_t k = 0; k < 3; ++k) {
      const std::uint32_t idx = poly.vertices[k];
      if (idx >= n_vertices) {
        continue;
      }
      const auto & v = vertices.points[idx];
      geometry_msgs::msg::Point p;
      p.x = v.x; p.y = v.y; p.z = v.z;
      tris.points.push_back(p);

      double t = 0.0;
      if (have_normals) {
        t = slope_to_ramp_t(vertex_normals.points[idx], style, slope_range);
      }
      tris.colors.push_back(slope_color(t, style.face_alpha));
    }
  }
  arr.markers.push_back(std::move(tris));

  // ---- Marker id=2: LINE_LIST (wireframe edges, light gray) ---------
  // Per triangle (a, b, c) emit 3 line segments: (a,b), (b,c), (c,a).
  // Shared edges between adjacent triangles get drawn twice — harmless
  // for visualization and saves the dedup bookkeeping.
  visualization_msgs::msg::Marker lines;
  lines.header.stamp = stamp;
  lines.header.frame_id = frame_id;
  lines.ns = "fast_mesh";
  lines.id = 2;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.pose.orientation.w = 1.0;
  // LINE_LIST: scale.x is line width in meters.
  lines.scale.x = style.edge_width_m;

  // Per-marker color (light gray) — applied to all segments since we
  // leave lines.colors empty.
  lines.color.r = 0.7f;
  lines.color.g = 0.7f;
  lines.color.b = 0.7f;
  lines.color.a = static_cast<float>(
    std::max(0.0, std::min(1.0, style.edge_alpha)));

  lines.points.reserve(mesh.polygons.size() * 6);

  auto push_edge = [&](std::uint32_t i0, std::uint32_t i1) {
    if (i0 >= n_vertices || i1 >= n_vertices) {
      return;
    }
    const auto & v0 = vertices.points[i0];
    const auto & v1 = vertices.points[i1];
    if (!std::isfinite(v0.x) || !std::isfinite(v0.y) || !std::isfinite(v0.z) ||
        !std::isfinite(v1.x) || !std::isfinite(v1.y) || !std::isfinite(v1.z))
    {
      return;
    }
    geometry_msgs::msg::Point p0, p1;
    p0.x = v0.x; p0.y = v0.y; p0.z = v0.z;
    p1.x = v1.x; p1.y = v1.y; p1.z = v1.z;
    lines.points.push_back(p0);
    lines.points.push_back(p1);
  };

  for (const auto & poly : mesh.polygons) {
    if (poly.vertices.size() != 3) {
      continue;
    }
    const std::uint32_t a = poly.vertices[0];
    const std::uint32_t b = poly.vertices[1];
    const std::uint32_t c = poly.vertices[2];
    push_edge(a, b);
    push_edge(b, c);
    push_edge(c, a);
  }
  arr.markers.push_back(std::move(lines));

  return arr;
}

}  // namespace realsense_fast_mesh_baseline
