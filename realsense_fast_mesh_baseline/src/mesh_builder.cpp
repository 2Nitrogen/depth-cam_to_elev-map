#include "realsense_fast_mesh_baseline/mesh_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

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
  const std::uint32_t stride       = std::max(1u, params.pixel_stride);
  const std::uint32_t cloud_width  = orig_width  / stride;
  const std::uint32_t cloud_height = orig_height / stride;

  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width    = cloud_width;
  cloud->height   = cloud_height;
  cloud->is_dense = false;
  cloud->points.resize(static_cast<std::size_t>(cloud_width) * cloud_height);

  // Intrinsics from CameraInfo.k (row-major 3x3). Applied to the
  // ORIGINAL pixel coordinates so back-projection is geometrically
  // correct regardless of stride.
  const float fx = static_cast<float>(camera_info.k[0]);
  const float fy = static_cast<float>(camera_info.k[4]);
  const float cx = static_cast<float>(camera_info.k[2]);
  const float cy = static_cast<float>(camera_info.k[5]);

  const float nan = std::numeric_limits<float>::quiet_NaN();

  if (!(fx > 0.0f) || !(fy > 0.0f)) {
    for (auto & pt : cloud->points) {
      pt.x = nan; pt.y = nan; pt.z = nan;
    }
    return cloud;
  }

  // Squared spherical cutoff threshold (avoids sqrt in the hot loop).
  // +inf disables the check when max_distance_m ≤ 0.
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
          const float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
          if (dist_sq > max_distance_sq) {
            pt.x = nan; pt.y = nan; pt.z = nan;
          }
        }
        pt.data[3] = 1.0f;
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
          const float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
          if (dist_sq > max_distance_sq) {
            pt.x = nan; pt.y = nan; pt.z = nan;
          }
        }
        pt.data[3] = 1.0f;
      }
    }
  } else {
    throw std::runtime_error(
      "Unsupported depth image encoding: '" + enc +
      "'. Supported: 16UC1, mono16, 32FC1.");
  }

  return cloud;
}

}  // namespace realsense_fast_mesh_baseline
