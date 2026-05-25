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

// Per-cell layered map state, all flattened row-major (idx = ix * size_y + iy).
//
//   height_map    : 1D Kalman estimate of cell elevation (m), NaN if empty.
//   variance_map  : posterior variance of that estimate (m^2), NaN if empty.
//   timestamp_map : seconds (matched to the node's clock) of last update,
//                   NaN if empty.
//   count_map     : number of measurements ever fused into the cell.
struct MapLayers
{
  std::vector<float>  height_map;
  std::vector<float>  variance_map;
  std::vector<double> timestamp_map;
  std::vector<int>    count_map;
};

// Tuning knobs for the per-cell fusion model. See fuse_point_into_cell for
// how each is used.
struct FusionParams
{
  // Measurement noise variance (m^2). For v1 this is a constant; a
  // range-dependent model (sensor_variance_base + k * range^2) is a natural
  // upgrade once we re-introduce p_camera coordinates per point.
  double sensor_variance{0.0009};

  // Posterior variance is clamped into [min_variance, max_variance] after
  // every fusion step. min_variance prevents over-confident cells from
  // refusing any future correction; max_variance is mostly a sanity rail.
  double min_variance{0.000009};
  double max_variance{0.01};

  // Mahalanobis distance threshold (in sigma) for accepting a measurement
  // into the Kalman update. Anything above this is routed to the
  // multi-height / outlier policy.
  double mahalanobis_threshold{2.5};

  // Variance bump for "different scan window" outliers — admits future
  // correction without thrashing the estimate.
  double multi_height_noise{0.0000009};

  // Time window for the "same scan" multi-height policy (s). A measurement
  // outside Mahalanobis is treated as part of the *same* scan if its age
  // relative to the cell's last update is within this window.
  double scanning_duration_sec{0.5};
};

// Allocate/reset the layer vectors to match `spec`. height/variance/timestamp
// become NaN, count becomes 0.
void reset_layers(const GridSpec & spec, MapLayers & layers);

// Drop cells whose timestamp is older than (now_sec - max_age_sec). The cells
// are reset to the empty state (NaN, count=0). Cells with NaN timestamps are
// left untouched (they're already empty).
void prune_stale_cells(MapLayers & layers, double now_sec, double max_age_sec);

// Fuse a single point's z value into the cell at (p.x, p.y).
//
// Behavior:
//   * Out-of-grid points return false; nothing else changes.
//   * Empty cell -> initialize (z=p.z, variance=sensor_variance, count=1).
//   * Existing cell -> Mahalanobis test:
//       d = |p.z - cell.z| / sqrt(cell.variance)
//     If d <= mahalanobis_threshold: 1D Kalman fusion
//       v_new = (v * v_meas) / (v + v_meas)
//       z_new = (v * p.z + v_meas * z) / (v + v_meas)
//     Else (outlier / multi-height): if the cell update is within
//     scanning_duration_sec of stamp_sec, the policy is "ignore-if-lower,
//     replace-if-higher" (transient occluder vs new obstacle on top); if
//     the cell is older, variance is bumped by multi_height_noise to admit
//     future correction.
//   * Variance is always clamped to [min_variance, max_variance].
bool fuse_point_into_cell(
  const pcl::PointXYZ & p,
  double stamp_sec,
  const GridSpec & spec,
  const FusionParams & params,
  MapLayers & layers);

// Convenience: fuse every point of a cloud into the layers.
void fuse_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  double stamp_sec,
  const GridSpec & spec,
  const FusionParams & params,
  MapLayers & layers);

// Materialize occupied cells (count >= min_points_per_cell) as a PointXYZI
// cloud: (x_center, y_center, height, intensity = sqrt(variance)). The
// 1-sigma uncertainty is exposed through intensity so RViz can color cells
// by confidence without an extra topic.
pcl::PointCloud<pcl::PointXYZI>::Ptr grid_to_xyzi_points(
  const GridSpec & spec,
  const MapLayers & layers,
  int min_points_per_cell);

}  // namespace realsense_elevation_mapper
