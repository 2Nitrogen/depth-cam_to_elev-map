#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace realsense_elevation_mapper
{

// ---------------------------------------------------------------------------
// Grid geometry
// ---------------------------------------------------------------------------
//
// Axis-aligned 2D elevation grid in the target frame's XY plane.
//
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

// ---------------------------------------------------------------------------
// Per-cell map state
// ---------------------------------------------------------------------------
//
// All flattened row-major (idx = ix * size_y + iy).
//
//   height_map    : 1D Kalman estimate of cell elevation (m), NaN if empty.
//   variance_map  : posterior variance of that estimate (m^2), NaN if empty.
//                   This is the MAP ESTIMATE variance P, distinct from the
//                   per-cell measurement variance R consumed during fusion.
//   timestamp_map : seconds (matched to the node's clock) of last update,
//                   NaN if empty.
//   count_map     : number of TEMPORAL updates that have hit this cell
//                   (i.e., how many frames have contributed an aggregated
//                   measurement to it). int64 so persistent non-cleanup
//                   maps don't overflow on long runs.
struct MapLayers
{
  std::vector<float>        height_map;
  std::vector<float>        variance_map;
  std::vector<double>       timestamp_map;
  std::vector<std::int64_t> count_map;
};

// ---------------------------------------------------------------------------
// Per-point measurement variance R
// ---------------------------------------------------------------------------
//
//   Constant     : R = sensor_variance (constant for every point — legacy
//                  behavior, useful for regression).
//   DepthSquared : R = sensor_noise_factor * depth^2. `depth` comes from
//                  the raw sensor-frame point (see DepthSource). Standard
//                  RealSense D435 noise model.
enum class MeasurementVarianceModel
{
  Constant,
  DepthSquared,
};

// Where the `depth` term comes from in the DepthSquared model.
//   RawZ      : depth = raw_point.z (RealSense optical convention).
//   RangeNorm : depth = sqrt(x^2 + y^2 + z^2) of the raw sensor-frame point.
enum class DepthSource
{
  RawZ,
  RangeNorm,
};

// Tuning knobs for fusion. See fuse_binned_frame for how each is used.
struct FusionParams
{
  // --- Per-point measurement variance R (before per-cell averaging) ---
  MeasurementVarianceModel measurement_variance_model{MeasurementVarianceModel::Constant};
  DepthSource depth_source{DepthSource::RawZ};
  double sensor_variance{0.0009};
  double sensor_noise_factor{0.0009};
  double min_measurement_variance{0.000009};
  double max_measurement_variance{0.01};

  // --- Mahalanobis gate (cell-level, applied per fused measurement) ---
  bool   mahalanobis_use_measurement_variance{false};
  double mahalanobis_threshold{2.5};
  // Variance bump applied to the *map estimate* P when the gate rejects a
  // measurement (= soft outlier handling, cupy-style).
  double multi_height_noise{0.0000009};

  // --- Map estimate variance P (clamped after every fusion step) ---
  double min_variance{0.000009};
  double max_variance{0.01};
};

// A single height measurement carrying the raw point information needed by
// the binning stage: target-frame (x, y, z) and the matching per-point R.
// Bundling all four removes any risk of index drift between parallel arrays.
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

// ---------------------------------------------------------------------------
// Cell-level measurement (output of stage ❶: binning)
// ---------------------------------------------------------------------------
//
// One CellMeasurement summarises every raw point of a single frame that
// landed in one grid cell:
//   * (x, y) are stored as world-frame coordinates of the cell centre,
//     NOT a packed cell_idx. This keeps the structure independent of any
//     particular GridSpec — the fusion stage re-locates the measurement
//     into the *current* spec, so the binning grid and the fusion grid do
//     not have to be identical (matters in inspection mode where the grid
//     auto-fits each callback).
//   * z is the mean z of the contributing raw points.
//   * variance is the mean R of those points.
//   * n_points is how many raw points were averaged (diagnostic).
struct CellMeasurement
{
  float         x;
  float         y;
  float         z;
  double        variance;
  std::uint32_t n_points;
};

// Deque element. Holds one frame's binning result + its arrival timestamp.
struct BinnedFrame
{
  double                       stamp_sec;
  std::vector<CellMeasurement> cells;  // sparse: only occupied cells
};

// ---------------------------------------------------------------------------
// Stage ❶: bin one frame's raw measurements into per-cell aggregates.
//
//   Iterates over `measurements` exactly once. For each point, locates its
//   cell in `spec`, accumulates (sum_z, sum_R, n) in a dense buffer, then
//   packs the non-empty cells into a sparse CellMeasurement vector. Within-
//   cell aggregation rule: arithmetic mean of z and of R (cupy-style — no
//   /N reduction of R; we do not claim the within-frame samples are i.i.d.).
//
//   Hot loop: plain arithmetic + integer indexing, no per-point function
//   calls. Suitable for ~10^4 input points per call.
std::vector<CellMeasurement> bin_frame_into_cells(
  const std::vector<HeightMeasurement> & measurements,
  const GridSpec & spec);

// ---------------------------------------------------------------------------
// Stage ❸: fuse one binned frame into the persistent layers.
//
//   For each CellMeasurement c in `cells`:
//     1. Locate its cell in `spec` from (c.x, c.y); skip if out of grid.
//     2. Empty cell -> initialise (z, R clamped into P range, count = 1).
//     3. Otherwise: Mahalanobis gate
//          gate_var = (use_R) ? (P + R) : P
//          d = |z_meas - z_cur| / sqrt(max(gate_var, min_variance))
//        d <= threshold -> standard 1D Kalman update.
//        d  > threshold -> bump P by multi_height_noise, keep z (cupy's
//                          soft outlier handling).
//     4. Always refresh timestamp and increment count.
//
//   Inner loop has at most three branches, no function calls. Cell count
//   per frame is typically O(10^3), so the whole loop costs ~µs.
void fuse_binned_frame(
  const std::vector<CellMeasurement> & cells,
  double stamp_sec,
  const GridSpec & spec,
  const FusionParams & params,
  MapLayers & layers);

// ---------------------------------------------------------------------------
// Layer lifecycle helpers
// ---------------------------------------------------------------------------

// Allocate/reset the layer vectors to match `spec`. height/variance/timestamp
// become NaN, count becomes 0.
void reset_layers(const GridSpec & spec, MapLayers & layers);

// Reset cells whose last update is older than (now_sec - max_age_sec). Used
// when continuous_cleanup is disabled and the map persists across callbacks.
void prune_stale_cells(MapLayers & layers, double now_sec, double max_age_sec);

// Materialize occupied cells (count >= min_points_per_cell) as a PointXYZI
// cloud: (x_center, y_center, height, intensity = sqrt(map estimate variance)).
pcl::PointCloud<pcl::PointXYZI>::Ptr grid_to_xyzi_points(
  const GridSpec & spec,
  const MapLayers & layers,
  int min_points_per_cell);

}  // namespace realsense_elevation_mapper
