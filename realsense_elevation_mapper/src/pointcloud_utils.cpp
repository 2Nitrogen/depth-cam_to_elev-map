#include "realsense_elevation_mapper/pointcloud_utils.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

namespace realsense_elevation_mapper
{

void crop_sphere(
  pcl::PointCloud<pcl::PointXYZ> & cloud,
  float max_distance)
{
  if (!(max_distance > 0.0f)) {
    return;  // cutoff disabled
  }
  const float r2 = max_distance * max_distance;

  pcl::PointCloud<pcl::PointXYZ> filtered;
  filtered.points.reserve(cloud.points.size());
  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const float d2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (d2 <= r2) {
      filtered.points.push_back(p);
    }
  }
  filtered.width = static_cast<std::uint32_t>(filtered.points.size());
  filtered.height = 1;
  filtered.is_dense = true;
  cloud = std::move(filtered);
}

}  // namespace realsense_elevation_mapper
