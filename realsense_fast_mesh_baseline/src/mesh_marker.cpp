#include "realsense_fast_mesh_baseline/mesh_marker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

// Slope angle (rad) of a unit-or-near-unit normal relative to +z. NaN-
// safe: returns 0 (flat) for degenerate / NaN normals.
double slope_from_normal(const Eigen::Vector3f & n)
{
  if (!n.allFinite()) {
    return 0.0;
  }
  const double mag = n.norm();
  if (!(mag > 0.0)) {
    return 0.0;
  }
  const double cos_abs = std::min(1.0, std::abs(static_cast<double>(n.z()) / mag));
  return std::acos(cos_abs);
}

double slope_to_ramp_t(
  const Eigen::Vector3f & n,
  const MarkerStyle & style,
  double slope_range)
{
  return (slope_from_normal(n) - style.color_min_slope_rad) / slope_range;
}

}  // namespace

visualization_msgs::msg::MarkerArray mesh_to_marker_array(
  const pcl::PolygonMesh & mesh,
  const std::vector<Eigen::Vector3f> & face_normals,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const MarkerStyle & style)
{
  visualization_msgs::msg::MarkerArray arr;

  // Extract vertices from PCL's serialized cloud format into an in-
  // memory PointCloud<PointXYZ> for index-based access.
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  const std::size_t n_vertices = vertices.points.size();
  const std::size_t n_polys    = mesh.polygons.size();
  const bool have_face_normals = (face_normals.size() == n_polys);

  const double slope_range =
    std::max(1e-6, style.color_max_slope_rad - style.color_min_slope_rad);

  // ---- Per-vertex aggregated normal (for POINTS Gouraud color) ------
  // Each vertex accumulates the (face_normal) of every triangle that
  // references it. A single pass over polygons populates the sums; we
  // do not normalize per-vertex (slope_from_normal handles arbitrary
  // magnitudes), so this is mathematically a weighted-by-incidence
  // average direction.
  std::vector<Eigen::Vector3f> vertex_normal_sums(n_vertices, Eigen::Vector3f::Zero());
  if (have_face_normals) {
    for (std::size_t fi = 0; fi < n_polys; ++fi) {
      const auto & poly = mesh.polygons[fi];
      if (poly.vertices.size() != 3) {
        continue;
      }
      const Eigen::Vector3f & nf = face_normals[fi];
      if (!nf.allFinite()) {
        continue;
      }
      for (std::size_t k = 0; k < 3; ++k) {
        const std::uint32_t idx = poly.vertices[k];
        if (idx < n_vertices) {
          vertex_normal_sums[idx] += nf;
        }
      }
    }
  }

  // ---- Marker id=0: POINTS (vertices, opaque, Gouraud color) --------
  visualization_msgs::msg::Marker pts;
  pts.header.stamp = stamp;
  pts.header.frame_id = frame_id;
  pts.ns = "fast_mesh";
  pts.id = 0;
  pts.type = visualization_msgs::msg::Marker::POINTS;
  pts.action = visualization_msgs::msg::Marker::ADD;
  pts.pose.orientation.w = 1.0;
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
    if (have_face_normals) {
      t = slope_to_ramp_t(vertex_normal_sums[i], style, slope_range);
    }
    pts.colors.push_back(slope_color(t, 1.0));  // opaque
  }
  arr.markers.push_back(std::move(pts));

  // ---- Marker id=1: TRIANGLE_LIST (faces, flat shading, faint) ------
  visualization_msgs::msg::Marker tris;
  tris.header.stamp = stamp;
  tris.header.frame_id = frame_id;
  tris.ns = "fast_mesh";
  tris.id = 1;
  tris.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  tris.action = visualization_msgs::msg::Marker::ADD;
  tris.pose.orientation.w = 1.0;
  tris.scale.x = 1.0;
  tris.scale.y = 1.0;
  tris.scale.z = 1.0;

  tris.points.reserve(n_polys * 3);
  tris.colors.reserve(n_polys * 3);

  for (std::size_t fi = 0; fi < n_polys; ++fi) {
    const auto & poly = mesh.polygons[fi];
    if (poly.vertices.size() != 3) {
      continue;
    }
    // Per-face color: one slope_color computed once from the face's
    // normal, then assigned to all 3 vertices of the triangle (flat
    // shading).
    double t = 0.0;
    if (have_face_normals) {
      t = slope_to_ramp_t(face_normals[fi], style, slope_range);
    }
    const auto face_col = slope_color(t, style.face_alpha);

    for (std::size_t k = 0; k < 3; ++k) {
      const std::uint32_t idx = poly.vertices[k];
      if (idx >= n_vertices) {
        continue;
      }
      const auto & v = vertices.points[idx];
      geometry_msgs::msg::Point p;
      p.x = v.x; p.y = v.y; p.z = v.z;
      tris.points.push_back(p);
      tris.colors.push_back(face_col);
    }
  }
  arr.markers.push_back(std::move(tris));

  // ---- Marker id=2: LINE_LIST (wireframe edges, light gray) ---------
  visualization_msgs::msg::Marker lines;
  lines.header.stamp = stamp;
  lines.header.frame_id = frame_id;
  lines.ns = "fast_mesh";
  lines.id = 2;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.pose.orientation.w = 1.0;
  lines.scale.x = style.edge_width_m;

  lines.color.r = 0.7f;
  lines.color.g = 0.7f;
  lines.color.b = 0.7f;
  lines.color.a = static_cast<float>(
    std::max(0.0, std::min(1.0, style.edge_alpha)));

  lines.points.reserve(n_polys * 6);

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
