#include "realsense_elevation_mapper/pointcloud_utils.hpp"

#include <utility>

namespace realsense_elevation_mapper
{

void crop_roi(
  pcl::PointCloud<pcl::PointXYZ> & cloud,
  float x_min, float x_max,
  float y_min, float y_max,
  float z_min, float z_max)
{
  pcl::PointCloud<pcl::PointXYZ> filtered;
  filtered.points.reserve(cloud.points.size());
  for (const auto & p : cloud.points) {
    if (p.x >= x_min && p.x < x_max &&
        p.y >= y_min && p.y < y_max &&
        p.z >= z_min && p.z < z_max)
    {
      filtered.points.push_back(p);
    }
  }
  filtered.width = static_cast<std::uint32_t>(filtered.points.size());
  filtered.height = 1;
  filtered.is_dense = cloud.is_dense;
  cloud = std::move(filtered);
}

}  // namespace realsense_elevation_mapper
