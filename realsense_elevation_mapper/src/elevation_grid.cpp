#include "realsense_elevation_mapper/elevation_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace realsense_elevation_mapper
{

// ---------------------------------------------------------------------------
// Grid spec
// ---------------------------------------------------------------------------

GridSpec make_grid_spec(
  double x_min, double x_max,
  double y_min, double y_max,
  double resolution)
{
  GridSpec spec;
  spec.x_min = x_min;
  spec.y_min = y_min;
  spec.resolution = resolution;
  spec.size_x = static_cast<std::size_t>(std::floor((x_max - x_min) / resolution));
  spec.size_y = static_cast<std::size_t>(std::floor((y_max - y_min) / resolution));
  return spec;
}

// ---------------------------------------------------------------------------
// Per-point measurement variance R
// ---------------------------------------------------------------------------

double compute_measurement_variance(
  const pcl::PointXYZ & raw_point,
  const FusionParams & params)
{
  double v = params.sensor_variance;

  if (params.measurement_variance_model == MeasurementVarianceModel::DepthSquared) {
    double depth = 0.0;
    if (params.depth_source == DepthSource::RawZ) {
      depth = static_cast<double>(raw_point.z);
    } else {
      const double rx = static_cast<double>(raw_point.x);
      const double ry = static_cast<double>(raw_point.y);
      const double rz = static_cast<double>(raw_point.z);
      depth = std::sqrt(rx * rx + ry * ry + rz * rz);
    }

    if (!std::isfinite(depth) || depth <= 0.0) {
      v = params.sensor_variance;  // non-physical depth → fallback
    } else {
      v = params.sensor_noise_factor * depth * depth;
    }
  }

  if (!std::isfinite(v) || v <= 0.0) {
    v = params.sensor_variance;
  }

  return std::clamp(v,
                    params.min_measurement_variance,
                    params.max_measurement_variance);
}

// ---------------------------------------------------------------------------
// Stage ❶: bin one frame into per-cell aggregates
// ---------------------------------------------------------------------------

namespace
{
struct CellAccum
{
  double        sum_z{0.0};
  double        sum_v{0.0};
  std::uint32_t n{0};
};
}  // namespace

std::vector<CellMeasurement> bin_frame_into_cells(
  const std::vector<HeightMeasurement> & measurements,
  const GridSpec & spec)
{
  const std::size_t n_cells = spec.size_x * spec.size_y;
  std::vector<CellMeasurement> out;
  if (n_cells == 0 || measurements.empty()) {
    return out;
  }

  std::vector<CellAccum> accum(n_cells);  // dense, zero-initialised
  const double inv_res = 1.0 / spec.resolution;

  // O(N) accumulation pass — no function calls in the loop body.
  for (const auto & m : measurements) {
    const double dx = (static_cast<double>(m.x) - spec.x_min) * inv_res;
    const double dy = (static_cast<double>(m.y) - spec.y_min) * inv_res;
    if (dx < 0.0 || dy < 0.0) {
      continue;
    }
    const std::size_t ix = static_cast<std::size_t>(dx);
    const std::size_t iy = static_cast<std::size_t>(dy);
    if (ix >= spec.size_x || iy >= spec.size_y) {
      continue;
    }
    auto & a = accum[ix * spec.size_y + iy];
    a.sum_z += static_cast<double>(m.z);
    a.sum_v += m.variance;
    a.n     += 1;
  }

  // Pack occupied cells into a sparse output. Output stores world-frame
  // cell centres so the fusion stage can use a different spec.
  out.reserve(n_cells / 8);  // heuristic: ~10% occupancy
  for (std::size_t ix = 0; ix < spec.size_x; ++ix) {
    const float x_center =
      static_cast<float>(spec.x_min + (static_cast<double>(ix) + 0.5) * spec.resolution);
    for (std::size_t iy = 0; iy < spec.size_y; ++iy) {
      const auto & a = accum[ix * spec.size_y + iy];
      if (a.n == 0) {
        continue;
      }
      const float  y_center =
        static_cast<float>(spec.y_min + (static_cast<double>(iy) + 0.5) * spec.resolution);
      const double inv_n   = 1.0 / static_cast<double>(a.n);
      out.push_back({
        x_center,
        y_center,
        static_cast<float>(a.sum_z * inv_n),
        a.sum_v * inv_n,
        a.n,
      });
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Stage ❸: fuse one binned frame into persistent layers
// ---------------------------------------------------------------------------

void fuse_binned_frame(
  const std::vector<CellMeasurement> & cells,
  double stamp_sec,
  const GridSpec & spec,
  const FusionParams & params,
  MapLayers & layers)
{
  if (cells.empty() || layers.height_map.empty()) {
    return;
  }

  // Hoist params out of the loop so the inner body is plain arithmetic.
  const double v_min_meas = params.min_measurement_variance;
  const double v_max_meas = params.max_measurement_variance;
  const double v_min_map  = params.min_variance;
  const double v_max_map  = params.max_variance;
  const double m_thresh   = params.mahalanobis_threshold;
  const bool   use_R_gate = params.mahalanobis_use_measurement_variance;
  const double v_bump     = params.multi_height_noise;
  const double inv_res    = 1.0 / spec.resolution;

  for (const auto & c : cells) {
    // Locate this cell-measurement in the *current* spec. We stored world
    // coordinates at bin time, so this works even if spec drifted since.
    const double dx = (static_cast<double>(c.x) - spec.x_min) * inv_res;
    const double dy = (static_cast<double>(c.y) - spec.y_min) * inv_res;
    if (dx < 0.0 || dy < 0.0) {
      continue;
    }
    const std::size_t ix = static_cast<std::size_t>(dx);
    const std::size_t iy = static_cast<std::size_t>(dy);
    if (ix >= spec.size_x || iy >= spec.size_y) {
      continue;
    }
    const std::size_t idx = ix * spec.size_y + iy;

    const double z_meas = static_cast<double>(c.z);
    const double v_meas = std::clamp(c.variance, v_min_meas, v_max_meas);

    if (layers.count_map[idx] == 0) {
      // First measurement to hit this cell.
      layers.height_map[idx]    = static_cast<float>(z_meas);
      layers.variance_map[idx]  = static_cast<float>(
        std::clamp(v_meas, v_min_map, v_max_map));
      layers.timestamp_map[idx] = stamp_sec;
      layers.count_map[idx]     = 1;
      continue;
    }

    const double z_curr   = layers.height_map[idx];
    const double v_curr   = layers.variance_map[idx];
    const double gate_v   = use_R_gate ? (v_curr + v_meas) : v_curr;
    const double safe_gate = std::max(gate_v, v_min_map);
    const double maha     = std::abs(z_meas - z_curr) / std::sqrt(safe_gate);

    if (maha <= m_thresh) {
      // Standard 1D Kalman fusion.
      const double v_sum = v_curr + v_meas;
      const double v_new = (v_curr * v_meas) / v_sum;
      const double z_new = (v_curr * z_meas + v_meas * z_curr) / v_sum;
      layers.height_map[idx]   = static_cast<float>(z_new);
      layers.variance_map[idx] = static_cast<float>(
        std::clamp(v_new, v_min_map, v_max_map));
    } else {
      // Outlier: bump P to admit future correction, keep z (cupy-style).
      const double v_bumped = v_curr + v_bump;
      layers.variance_map[idx] = static_cast<float>(
        std::clamp(v_bumped, v_min_map, v_max_map));
    }

    layers.timestamp_map[idx] = stamp_sec;
    layers.count_map[idx]    += 1;
  }
}

// ---------------------------------------------------------------------------
// Layer lifecycle
// ---------------------------------------------------------------------------

void reset_layers(const GridSpec & spec, MapLayers & layers)
{
  const std::size_t n = spec.size_x * spec.size_y;
  const float  fnan = std::numeric_limits<float>::quiet_NaN();
  const double dnan = std::numeric_limits<double>::quiet_NaN();
  layers.height_map.assign(n, fnan);
  layers.variance_map.assign(n, fnan);
  layers.timestamp_map.assign(n, dnan);
  layers.count_map.assign(n, 0);
}

void prune_stale_cells(MapLayers & layers, double now_sec, double max_age_sec)
{
  if (!(max_age_sec > 0.0)) {
    return;
  }
  const std::size_t n = layers.count_map.size();
  const float  fnan = std::numeric_limits<float>::quiet_NaN();
  const double dnan = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t i = 0; i < n; ++i) {
    if (layers.count_map[i] == 0) {
      continue;
    }
    const double t = layers.timestamp_map[i];
    if (!std::isfinite(t)) {
      continue;
    }
    if (now_sec - t > max_age_sec) {
      layers.height_map[i]    = fnan;
      layers.variance_map[i]  = fnan;
      layers.timestamp_map[i] = dnan;
      layers.count_map[i]     = 0;
    }
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr grid_to_xyzi_points(
  const GridSpec & spec,
  const MapLayers & layers,
  int min_points_per_cell)
{
  auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  out->points.reserve(spec.size_x * spec.size_y / 4);
  for (std::size_t ix = 0; ix < spec.size_x; ++ix) {
    for (std::size_t iy = 0; iy < spec.size_y; ++iy) {
      const std::size_t idx = ix * spec.size_y + iy;
      if (layers.count_map[idx] < min_points_per_cell) {
        continue;
      }
      pcl::PointXYZI p;
      p.x = static_cast<float>(spec.x_min + (static_cast<double>(ix) + 0.5) * spec.resolution);
      p.y = static_cast<float>(spec.y_min + (static_cast<double>(iy) + 0.5) * spec.resolution);
      p.z = layers.height_map[idx];
      const float v = layers.variance_map[idx];
      p.intensity = std::sqrt(std::max(v, 0.0f));
      out->points.push_back(p);
    }
  }
  out->width = static_cast<std::uint32_t>(out->points.size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

}  // namespace realsense_elevation_mapper
