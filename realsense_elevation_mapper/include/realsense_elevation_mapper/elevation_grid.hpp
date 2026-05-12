#pragma once

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace realsense_elevation_mapper
{

struct GridSpec
{
  double x_min{0.0};
  double y_min{0.0};
  double resolution{0.02};
  std::size_t size_x{0};
  std::size_t size_y{0};
};

GridSpec make_grid_spec(
  double x_min, double x_max,
  double y_min, double y_max,
  double resolution);

// Bin every point into the grid. Returns height_map and count_map flattened
// row-major with stride size_y. Cells with count == 0 hold NaN in
// height_map. height_map and count_map are reset on each call.
void compute_mean_elevation(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const GridSpec & spec,
  std::vector<float> & height_map,
  std::vector<int> & count_map);

// Convert valid cells (count >= min_points_per_cell) to a point cloud at the
// cell centers with z = mean elevation.
pcl::PointCloud<pcl::PointXYZ>::Ptr grid_to_points(
  const std::vector<float> & height_map,
  const std::vector<int> & count_map,
  const GridSpec & spec,
  int min_points_per_cell);

}  // namespace realsense_elevation_mapper
