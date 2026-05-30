#include "realsense_fast_mesh_baseline/mesh_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <pcl/conversions.h>
#include <pcl/surface/organized_fast_mesh.h>

#include <sensor_msgs/image_encodings.hpp>

namespace realsense_fast_mesh_baseline
{

pcl::PointCloud<pcl::PointXYZ>::Ptr build_organized_cloud_from_depth(
  const sensor_msgs::msg::Image & depth_image,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const MeshBuilderParams & params)
{
  const std::uint32_t orig_width  = depth_image.width;
  const std::uint32_t orig_height = depth_image.height;
  // Defensive clamp on stride. stride>orig_dim would produce a 0-sized
  // cloud which is harmless downstream but useless; we leave that as
  // the caller's responsibility for now and just guard against 0.
  const std::uint32_t stride       = std::max(1u, params.pixel_stride);
  const std::uint32_t cloud_width  = orig_width  / stride;
  const std::uint32_t cloud_height = orig_height / stride;

  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width    = cloud_width;
  cloud->height   = cloud_height;
  cloud->is_dense = false;   // NaN points are allowed (and expected at holes)
  cloud->points.resize(static_cast<std::size_t>(cloud_width) * cloud_height);

  // Intrinsics from CameraInfo.k (row-major 3x3):
  //   [ fx  0 cx ]
  //   [  0 fy cy ]
  //   [  0  0  1 ]
  // Intrinsics are calibrated against the ORIGINAL image, so the back-
  // projection formulas below use the original (u_orig, v_orig)
  // coordinates of each sampled pixel — never the downsampled index.
  const float fx = static_cast<float>(camera_info.k[0]);
  const float fy = static_cast<float>(camera_info.k[4]);
  const float cx = static_cast<float>(camera_info.k[2]);
  const float cy = static_cast<float>(camera_info.k[5]);

  const float nan = std::numeric_limits<float>::quiet_NaN();

  // Defensive guard against an empty / un-populated CameraInfo.
  if (!(fx > 0.0f) || !(fy > 0.0f)) {
    for (auto & pt : cloud->points) {
      pt.x = nan; pt.y = nan; pt.z = nan;
    }
    return cloud;
  }

  // Precompute the squared max-distance for the spherical cutoff so the
  // per-pixel test is a multiplication + comparison (no sqrt). When the
  // cutoff is disabled (<= 0), set the threshold to +inf so no point
  // ever exceeds it.
  const float max_distance_sq =
    (params.max_distance_m > 0.0f)
      ? params.max_distance_m * params.max_distance_m
      : std::numeric_limits<float>::infinity();

  const std::string & enc = depth_image.encoding;
  using namespace sensor_msgs::image_encodings;

  if (enc == TYPE_16UC1 || enc == MONO16) {
    // 16-bit unsigned, depth in millimeters (RealSense convention).
    const auto * data_base = reinterpret_cast<const std::uint16_t *>(depth_image.data.data());
    const std::size_t row_stride_px = depth_image.step / sizeof(std::uint16_t);
    for (std::uint32_t v_new = 0; v_new < cloud_height; ++v_new) {
      const std::uint32_t v_orig = v_new * stride;
      const auto * row = data_base + v_orig * row_stride_px;
      for (std::uint32_t u_new = 0; u_new < cloud_width; ++u_new) {
        const std::uint32_t u_orig = u_new * stride;
        const std::uint16_t d_mm = row[u_orig];
        auto & pt = cloud->points[v_new * cloud_width + u_new];
        if (d_mm == 0) {
          pt.x = nan; pt.y = nan; pt.z = nan;
        } else {
          const float d = static_cast<float>(d_mm) * 0.001f;
          pt.x = (static_cast<float>(u_orig) - cx) * d / fx;
          pt.y = (static_cast<float>(v_orig) - cy) * d / fy;
          pt.z = d;
          // Spherical range cutoff: drop points outside sqrt(x²+y²+z²)
          // > max_distance_m. Squared comparison avoids the sqrt.
          const float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
          if (dist_sq > max_distance_sq) {
            pt.x = nan; pt.y = nan; pt.z = nan;
          }
        }
        pt.data[3] = 1.0f;  // SSE alignment field — some PCL kernels read it
      }
    }
  } else if (enc == TYPE_32FC1) {
    // 32-bit float, depth in meters.
    const auto * data_base = reinterpret_cast<const float *>(depth_image.data.data());
    const std::size_t row_stride_px = depth_image.step / sizeof(float);
    for (std::uint32_t v_new = 0; v_new < cloud_height; ++v_new) {
      const std::uint32_t v_orig = v_new * stride;
      const auto * row = data_base + v_orig * row_stride_px;
      for (std::uint32_t u_new = 0; u_new < cloud_width; ++u_new) {
        const std::uint32_t u_orig = u_new * stride;
        const float d = row[u_orig];
        auto & pt = cloud->points[v_new * cloud_width + u_new];
        if (!std::isfinite(d) || d <= 0.0f) {
          pt.x = nan; pt.y = nan; pt.z = nan;
        } else {
          pt.x = (static_cast<float>(u_orig) - cx) * d / fx;
          pt.y = (static_cast<float>(v_orig) - cy) * d / fy;
          pt.z = d;
          // Spherical range cutoff (see 16UC1 branch for rationale).
          const float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
          if (dist_sq > max_distance_sq) {
            pt.x = nan; pt.y = nan; pt.z = nan;
          }
        }
        pt.data[3] = 1.0f;  // SSE alignment field — some PCL kernels read it
      }
    }
  } else {
    throw std::runtime_error(
      "Unsupported depth image encoding: '" + enc +
      "'. Supported: 16UC1, mono16, 32FC1.");
  }

  return cloud;
}


TriangulationType parse_triangulation_type(const std::string & s)
{
  if (s == "TRIANGLE_RIGHT_CUT")    { return TriangulationType::TriangleRightCut; }
  if (s == "TRIANGLE_LEFT_CUT")     { return TriangulationType::TriangleLeftCut; }
  if (s == "TRIANGLE_ADAPTIVE_CUT") { return TriangulationType::TriangleAdaptiveCut; }
  throw std::runtime_error(
    "Invalid triangulation_type '" + s +
    "'. Valid options: TRIANGLE_RIGHT_CUT, TRIANGLE_LEFT_CUT, "
    "TRIANGLE_ADAPTIVE_CUT. (QUAD_MESH not supported — Marker output "
    "is TRIANGLE_LIST only.)");
}

const char * triangulation_type_str(TriangulationType t)
{
  switch (t) {
    case TriangulationType::TriangleRightCut:    return "TRIANGLE_RIGHT_CUT";
    case TriangulationType::TriangleLeftCut:     return "TRIANGLE_LEFT_CUT";
    case TriangulationType::TriangleAdaptiveCut: return "TRIANGLE_ADAPTIVE_CUT";
  }
  return "?";
}

pcl::PolygonMesh build_fast_mesh(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const MeshBuilderParams & params)
{
  pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
  ofm.setInputCloud(cloud);
  ofm.setMaxEdgeLength(params.triangle_max_edge_length);

  using TT = pcl::OrganizedFastMesh<pcl::PointXYZ>;
  switch (params.triangulation_type) {
    case TriangulationType::TriangleRightCut:
      ofm.setTriangulationType(TT::TRIANGLE_RIGHT_CUT);
      break;
    case TriangulationType::TriangleLeftCut:
      ofm.setTriangulationType(TT::TRIANGLE_LEFT_CUT);
      break;
    case TriangulationType::TriangleAdaptiveCut:
      ofm.setTriangulationType(TT::TRIANGLE_ADAPTIVE_CUT);
      break;
  }

  pcl::PolygonMesh mesh;
  ofm.reconstruct(mesh);
  return mesh;
}

std::vector<Eigen::Vector3f> compute_face_normals(const pcl::PolygonMesh & mesh)
{
  // mesh.cloud is PCL's serialized PCLPointCloud2 — convert to a typed
  // PointCloud<PointXYZ> so we can index vertex coordinates.
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  const std::size_t n_vertices = vertices.points.size();
  std::vector<Eigen::Vector3f> normals;
  normals.reserve(mesh.polygons.size());

  const Eigen::Vector3f nan_n{
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()};

  for (const auto & poly : mesh.polygons) {
    // OrganizedFastMesh always emits triangles; defensive guard.
    if (poly.vertices.size() != 3) {
      normals.push_back(nan_n);
      continue;
    }
    const std::uint32_t ia = poly.vertices[0];
    const std::uint32_t ib = poly.vertices[1];
    const std::uint32_t ic = poly.vertices[2];
    if (ia >= n_vertices || ib >= n_vertices || ic >= n_vertices) {
      normals.push_back(nan_n);
      continue;
    }

    const auto & a = vertices.points[ia];
    const auto & b = vertices.points[ib];
    const auto & c = vertices.points[ic];

    // build_fast_mesh strips triangles whose vertices are NaN, so the
    // arithmetic below is generally safe. Still, defensive on size and
    // zero-area triangles below.
    const Eigen::Vector3f ea(b.x - a.x, b.y - a.y, b.z - a.z);
    const Eigen::Vector3f eb(c.x - a.x, c.y - a.y, c.z - a.z);
    const Eigen::Vector3f n = ea.cross(eb);
    const float mag = n.norm();
    if (!(mag > 0.0f) || !n.allFinite()) {
      normals.push_back(nan_n);
      continue;
    }
    normals.push_back(n / mag);
  }

  return normals;
}

}  // namespace realsense_fast_mesh_baseline
