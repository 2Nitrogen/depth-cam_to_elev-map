#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace realsense_elevation_mapper
{

// In-place ROI crop: keep only points strictly inside the half-open box
// [x_min, x_max) x [y_min, y_max) x [z_min, z_max). Matches the Python
// crop_roi semantics.
void crop_roi(
  pcl::PointCloud<pcl::PointXYZ> & cloud,
  float x_min, float x_max,
  float y_min, float y_max,
  float z_min, float z_max);

}  // namespace realsense_elevation_mapper
