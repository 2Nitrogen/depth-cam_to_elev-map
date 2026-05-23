#pragma once

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace realsense_elevation_mapper
{

// Axis-aligned 2D elevation grid in the target frame's XY plane.
//
// Cell layout:
//   size_x cells along X, size_y cells along Y.
//   Cell (ix, iy) covers
//     [x_min + ix     * resolution, x_min + (ix + 1) * resolution) in X
//     [y_min + iy     * resolution, y_min + (iy + 1) * resolution) in Y
//   Cell center: (x_min + (ix + 0.5) * resolution,
//                 y_min + (iy + 0.5) * resolution).
//
// Flattening: parallel std::vector<>'s of length size_x * size_y,
// row-major with stride size_y -> idx = ix * size_y + iy.
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

// Bin every point into the grid and write the per-cell arithmetic mean Z
// into `height_map`.
//
// Inputs:
//   cloud: an N-point PCL cloud already in the grid's coordinate frame.
//   spec : grid geometry (see GridSpec).
//
// Outputs (resized to size_x * size_y on each call):
//   height_map: float per cell. Cells with at least one point hold the
//               arithmetic mean of the points' Z (m). Empty cells hold
//               NaN (paired with count_map[i] == 0).
//   count_map : int per cell, number of points that fell in that cell.
//
// Index mapping (used internally):
//   ix = floor((p.x - x_min) / resolution)
//   iy = floor((p.y - y_min) / resolution)
//   idx = ix * size_y + iy   (row-major, stride size_y)
// Points with (ix, iy) outside [0, size_x) x [0, size_y) are silently
// dropped — callers that want every point counted must size the grid
// large enough first.
//
// Algorithm note: this is the current "single-statistic per cell" path
// (mean Z). To swap in a more robust estimator (median, trimmed mean,
// MAD-based outlier rejection) the natural replacement is a variant that
// keeps the per-cell point list (or running quantile sketches) and
// computes the estimate in a second pass.
void compute_mean_elevation(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const GridSpec & spec,
  std::vector<float> & height_map,
  std::vector<int> & count_map);

// Materialize every cell with count >= min_points_per_cell as a single
// point at the cell center: (x_center, y_center, height_map[idx]).
// The result is an unorganized PCL cloud of <= size_x * size_y points,
// safe to convert to sensor_msgs/PointCloud2 with pcl::toROSMsg.
pcl::PointCloud<pcl::PointXYZ>::Ptr grid_to_points(
  const std::vector<float> & height_map,
  const std::vector<int> & count_map,
  const GridSpec & spec,
  int min_points_per_cell);

}  // namespace realsense_elevation_mapper
