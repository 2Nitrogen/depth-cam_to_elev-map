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
//                   This is the MAP ESTIMATE variance P, distinct from the
//                   per-measurement variance R consumed during fusion.
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

// Per-point measurement variance model. See FusionParams.
//   Constant     : R = sensor_variance (constant for every point — legacy
//                  behavior, default).
//   DepthSquared : R = sensor_noise_factor * depth^2. `depth` comes from the
//                  raw sensor-frame point (see DepthSource).
enum class MeasurementVarianceModel
{
  Constant,
  DepthSquared,
};

// Where the `depth` term comes from in the DepthSquared model.
//   RawZ      : depth = raw_point.z (RealSense optical-frame convention:
//               z is the optical depth axis).
//   RangeNorm : depth = sqrt(x^2 + y^2 + z^2) of the raw sensor-frame point.
enum class DepthSource
{
  RawZ,
  RangeNorm,
};

// Tuning knobs for the per-cell fusion model. See fuse_measurement_into_cell
// for how each is used.
struct FusionParams
{
  // --- Measurement variance R (per-point, before fusion) ---
  // Selects between the two models above.
  MeasurementVarianceModel measurement_variance_model{MeasurementVarianceModel::Constant};
  // Which raw-frame depth to plug into the DepthSquared model.
  DepthSource depth_source{DepthSource::RawZ};
  // Constant measurement variance R (used as-is in Constant mode and as
  // fallback for non-finite / non-positive depth in DepthSquared mode).
  double sensor_variance{0.0009};
  // R = sensor_noise_factor * depth^2 in DepthSquared mode.
  double sensor_noise_factor{0.0009};
  // R is clamped into [min_measurement_variance, max_measurement_variance]
  // before each Kalman update.
  double min_measurement_variance{0.000009};
  double max_measurement_variance{0.01};
  // If true, the Mahalanobis gate uses innovation variance P + R; if false,
  // only P (default — matches pre-existing gating behavior).
  bool mahalanobis_use_measurement_variance{false};

  // --- Map estimate variance P (per-cell, posterior) ---
  // Posterior variance is clamped into [min_variance, max_variance] after
  // every fusion step. min_variance prevents over-confident cells from
  // refusing any future correction; max_variance is mostly a sanity rail.
  double min_variance{0.000009};
  double max_variance{0.01};

  // --- Multi-height / outlier policy ---
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

// A single depth measurement, ready to fuse into the grid.
//
// (x, y, z) are in the *target frame* (gravity-aligned for our use case)
// — they drive cell indexing and the Kalman update on z.
//
// `variance` is the measurement variance R, in m^2, computed from the
// matching raw sensor-frame point via compute_measurement_variance(). It
// is intended to be already clamped, but fuse_measurement_into_cell
// re-clamps defensively.
//
// Bundling all four fields in one struct removes any risk of
// raw/transformed index drift through buffers, ROI crops, and concat.
struct HeightMeasurement
{
  float  x;
  float  y;
  float  z;
  double variance;
};

// Compute the measurement variance R for one raw sensor-frame point.
//
// Constant mode      : returns params.sensor_variance (clamped).
// DepthSquared mode  : depth = raw_point.z (RawZ) or
//                              sqrt(x^2+y^2+z^2) (RangeNorm).
//                      R = sensor_noise_factor * depth^2.
//                      Falls back to sensor_variance for non-finite or
//                      non-positive depth.
// In all cases the result is clamped into
// [min_measurement_variance, max_measurement_variance].
double compute_measurement_variance(
  const pcl::PointXYZ & raw_point,
  const FusionParams & params);

// Allocate/reset the layer vectors to match `spec`. height/variance/timestamp
// become NaN, count becomes 0.
void reset_layers(const GridSpec & spec, MapLayers & layers);

// Drop cells whose timestamp is older than (now_sec - max_age_sec). The cells
// are reset to the empty state (NaN, count=0). Cells with NaN timestamps are
// left untouched (they're already empty).
void prune_stale_cells(MapLayers & layers, double now_sec, double max_age_sec);

// Fuse a single HeightMeasurement into its cell.
//
// Behavior:
//   * Out-of-grid measurements return false; nothing else changes.
//   * Empty cell -> initialize (z=m.z, variance=m.variance, count=1).
//   * Existing cell -> Mahalanobis test:
//       gate_variance = use_R ? P + R : P
//       d = |m.z - cell.z| / sqrt(max(gate_variance, params.min_variance))
//     If d <= mahalanobis_threshold: 1D Kalman fusion
//       v_new = (P * R) / (P + R)
//       z_new = (P * m.z + R * z) / (P + R)
//     Else (outlier / multi-height): if the cell update is within
//     scanning_duration_sec of stamp_sec, the policy is "ignore-if-lower,
//     replace-if-higher" (transient occluder vs new obstacle on top); if
//     the cell is older, variance is bumped by multi_height_noise.
//   * R is clamped to [min_measurement_variance, max_measurement_variance].
//   * P is clamped to [min_variance, max_variance] after every update.
bool fuse_measurement_into_cell(
  const HeightMeasurement & m,
  double stamp_sec,
  const GridSpec & spec,
  const FusionParams & params,
  MapLayers & layers);

// Convenience: fuse every measurement in the vector into the layers.
void fuse_measurements(
  const std::vector<HeightMeasurement> & measurements,
  double stamp_sec,
  const GridSpec & spec,
  const FusionParams & params,
  MapLayers & layers);

// Materialize occupied cells (count >= min_points_per_cell) as a PointXYZI
// cloud: (x_center, y_center, height, intensity = sqrt(variance)). The
// 1-sigma uncertainty (of the MAP estimate, not the per-point measurement)
// is exposed through intensity so RViz can color cells by confidence.
pcl::PointCloud<pcl::PointXYZI>::Ptr grid_to_xyzi_points(
  const GridSpec & spec,
  const MapLayers & layers,
  int min_points_per_cell);

}  // namespace realsense_elevation_mapper
