#include "realsense_fast_mesh_baseline/mesh_marker.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

namespace realsense_fast_mesh_baseline
{
namespace
{

std_msgs::msg::ColorRGBA slope_color(double t)
{
  t = std::max(0.0, std::min(1.0, t));
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(t);
  c.g = static_cast<float>(1.0 - t);
  c.b = 0.0f;
  c.a = 1.0f;
  return c;
}

// Slope (rad) of a vertex's normal relative to +z (gravity = world up
// in the gravity-aligned target frame). NaN-safe: returns 0 (flat) if
// the normal is degenerate.
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

}  // namespace

visualization_msgs::msg::Marker mesh_to_triangle_list_marker(
  const pcl::PolygonMesh & mesh,
  const pcl::PointCloud<pcl::Normal> & vertex_normals,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const MarkerStyle & style)
{
  visualization_msgs::msg::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id;
  m.ns = "fast_mesh";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;

  // Extract vertices from mesh.cloud (pcl::PCLPointCloud2 — PCL's own
  // serialized cloud format, NOT sensor_msgs/PointCloud2) into an
  // in-memory PCL cloud for index-based access.
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  const std::size_t n_vertices = vertices.points.size();
  const std::size_t n_normals  = vertex_normals.points.size();
  // Defensive: normals come from the camera-frame organized cloud, the
  // mesh's vertices come from the (possibly transformed) cloud. Both
  // should have identical width*height, but if not we just skip color.
  const bool have_normals = (n_normals == n_vertices);

  const double slope_range =
    std::max(1e-6, style.color_max_slope_rad - style.color_min_slope_rad);

  m.points.reserve(mesh.polygons.size() * 3);
  m.colors.reserve(mesh.polygons.size() * 3);

  for (const auto & poly : mesh.polygons) {
    if (poly.vertices.size() != 3) {
      continue;  // OrganizedFastMesh always emits triangles, but be defensive
    }
    for (std::size_t k = 0; k < 3; ++k) {
      const std::uint32_t idx = poly.vertices[k];
      if (idx >= n_vertices) {
        continue;
      }
      const auto & v = vertices.points[idx];
      geometry_msgs::msg::Point p;
      p.x = v.x;
      p.y = v.y;
      p.z = v.z;
      m.points.push_back(p);

      double t = 0.0;
      if (have_normals) {
        const double slope = slope_from_normal(vertex_normals.points[idx]);
        t = (slope - style.color_min_slope_rad) / slope_range;
      }
      m.colors.push_back(slope_color(t));
    }
  }

  return m;
}

}  // namespace realsense_fast_mesh_baseline
