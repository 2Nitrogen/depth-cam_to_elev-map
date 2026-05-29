#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace realsense_elevation_mapper
{

// In-place spherical cutoff: drop points whose distance from the
// origin (sqrt(x² + y² + z²)) exceeds `max_distance`. Designed to be
// called on the cloud in CAMERA optical frame (where the camera is at
// the origin), so the cutoff is a sphere centered on the sensor — the
// natural "depth-aware" trim for RGB-D data, since RealSense depth
// noise scales with depth².
//
// NaN / non-finite points are also dropped (defensive — the caller may
// have already filtered NaN via pcl::removeNaNFromPointCloud).
//
// If `max_distance` <= 0 the cutoff is disabled (no-op). After the
// call the cloud is unorganized (height = 1) and dense (is_dense = true).
void crop_sphere(
  pcl::PointCloud<pcl::PointXYZ> & cloud,
  float max_distance);

}  // namespace realsense_elevation_mapper
