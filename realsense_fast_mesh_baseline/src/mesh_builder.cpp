#include "realsense_fast_mesh_baseline/mesh_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

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

pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const MeshBuilderParams & params)
{
  // Manual per-pixel cross-product normal estimation on an organized
  // cloud. Replaces pcl::IntegralImageNormalEstimation, which aborts
  // with a `variable_if_dynamic<long, 3>` Eigen assertion on our PCL
  // 1.12 / Eigen 3.4 / 848x480 setup regardless of the estimation
  // method (AVERAGE_3D_GRADIENT, COVARIANCE_MATRIX, …). The manual
  // form has no PCL/Eigen internals to trip — just plain math.
  //
  // Algorithm:
  //   For each pixel (u, v), sample 3 points from the cloud:
  //       p_c = cloud(u,        v       )   (center)
  //       p_r = cloud(u + step, v       )   (right neighbor)
  //       p_d = cloud(u,        v + step)   (down  neighbor)
  //   Reject the pixel if any of p_c / p_r / p_d is NaN, or if the
  //   relative depth gap between center and a neighbor exceeds
  //   `max_depth_change_factor` (step-edge / discontinuity).
  //   Otherwise:
  //       n = (p_r - p_c) × (p_d - p_c)
  //       normalize(n)
  //   `step` is taken from `normal_smoothing_size` (in pixels) — larger
  //   step = smoother normals, less per-pixel sensor-noise sensitivity,
  //   at the cost of slower response to small features.
  //
  // Note on normal sign: the cross product convention (right × down)
  // yields a vector along the camera optical +z axis (into the scene)
  // for surfaces facing the camera, which is the OPPOSITE of the
  // convention used by pcl::IntegralImageNormalEstimation (normals
  // toward the camera). For slope-angle visualization this is harmless
  // (slope = arccos(|n_z|) is sign-agnostic), but downstream consumers
  // that care about orientation should be aware.
  auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  const int W = static_cast<int>(cloud->width);
  const int H = static_cast<int>(cloud->height);
  normals->width    = W;
  normals->height   = H;
  normals->is_dense = false;
  normals->points.resize(static_cast<std::size_t>(W) * H);

  const float nan = std::numeric_limits<float>::quiet_NaN();
  // Pre-fill all output normals with NaN; we'll overwrite the valid ones.
  for (auto & n : normals->points) {
    n.normal_x = nan; n.normal_y = nan; n.normal_z = nan;
    n.curvature = 0.0f;
  }

  // Reinterpret `normal_smoothing_size` (originally in px) as the
  // gradient sample step. 1 px = raw per-pixel normal (noisy); larger
  // = smoother normals via wider baseline.
  const int step = std::max(1, static_cast<int>(params.normal_smoothing_size));
  const float max_dz_factor = params.max_depth_change_factor;

  for (int v = 0; v + step < H; ++v) {
    for (int u = 0; u + step < W; ++u) {
      const auto & pc = cloud->points[v * W + u];
      const auto & pr = cloud->points[v * W + (u + step)];
      const auto & pd = cloud->points[(v + step) * W + u];

      // Need all three points to have finite, positive depth.
      if (!std::isfinite(pc.z) || pc.z <= 0.0f ||
          !std::isfinite(pr.z) || pr.z <= 0.0f ||
          !std::isfinite(pd.z) || pd.z <= 0.0f)
      {
        continue;
      }

      // Depth-discontinuity rejection (step edges).
      if (max_dz_factor > 0.0f) {
        const float dz_thresh = pc.z * max_dz_factor;
        if (std::abs(pr.z - pc.z) > dz_thresh ||
            std::abs(pd.z - pc.z) > dz_thresh)
        {
          continue;
        }
      }

      // Cross product of (right - center) × (down - center).
      const float dx_r = pr.x - pc.x, dy_r = pr.y - pc.y, dz_r = pr.z - pc.z;
      const float dx_d = pd.x - pc.x, dy_d = pd.y - pc.y, dz_d = pd.z - pc.z;

      const float nx = dy_r * dz_d - dz_r * dy_d;
      const float ny = dz_r * dx_d - dx_r * dz_d;
      const float nz = dx_r * dy_d - dy_r * dx_d;

      const float mag = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (mag <= 0.0f) {
        continue;
      }
      const float inv = 1.0f / mag;
      auto & n_out = normals->points[v * W + u];
      n_out.normal_x = nx * inv;
      n_out.normal_y = ny * inv;
      n_out.normal_z = nz * inv;
      n_out.curvature = 0.0f;
    }
  }

  return normals;
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

}  // namespace realsense_fast_mesh_baseline
