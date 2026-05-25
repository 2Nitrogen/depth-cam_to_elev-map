#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "realsense_elevation_mapper/elevation_grid.hpp"

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

// Same half-open box but for HeightMeasurement vectors (in-place). Used
// after we've already bundled raw-derived variance into the measurements,
// so we don't accidentally desynchronize a parallel variance vector.
void crop_measurements_roi(
  std::vector<HeightMeasurement> & measurements,
  float x_min, float x_max,
  float y_min, float y_max,
  float z_min, float z_max);

}  // namespace realsense_elevation_mapper
