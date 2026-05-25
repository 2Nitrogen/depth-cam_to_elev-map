#include "realsense_elevation_mapper/elevation_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace realsense_elevation_mapper
{

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

bool fuse_point_into_cell(
  const pcl::PointXYZ & p,
  double stamp_sec,
  const GridSpec & spec,
  const FusionParams & params,
  MapLayers & layers)
{
  const double dx = (static_cast<double>(p.x) - spec.x_min) / spec.resolution;
  const double dy = (static_cast<double>(p.y) - spec.y_min) / spec.resolution;
  if (dx < 0.0 || dy < 0.0) {
    return false;
  }
  const std::size_t ix = static_cast<std::size_t>(dx);
  const std::size_t iy = static_cast<std::size_t>(dy);
  if (ix >= spec.size_x || iy >= spec.size_y) {
    return false;
  }
  const std::size_t idx = ix * spec.size_y + iy;

  const double v_meas = params.sensor_variance;
  const double z_meas = static_cast<double>(p.z);
  const double clamped_meas =
    std::clamp(v_meas, params.min_variance, params.max_variance);

  if (layers.count_map[idx] == 0) {
    layers.height_map[idx]    = static_cast<float>(z_meas);
    layers.variance_map[idx]  = static_cast<float>(clamped_meas);
    layers.timestamp_map[idx] = stamp_sec;
    layers.count_map[idx]     = 1;
    return true;
  }

  const double z_curr = static_cast<double>(layers.height_map[idx]);
  const double v_curr = static_cast<double>(layers.variance_map[idx]);
  const double sigma_curr = std::sqrt(std::max(v_curr, params.min_variance));
  const double mahalanobis = std::abs(z_meas - z_curr) / sigma_curr;

  if (mahalanobis <= params.mahalanobis_threshold) {
    // 1D Kalman fusion.
    const double v_sum = v_curr + v_meas;
    const double v_new = (v_curr * v_meas) / v_sum;
    const double z_new = (v_curr * z_meas + v_meas * z_curr) / v_sum;
    layers.height_map[idx]   = static_cast<float>(z_new);
    layers.variance_map[idx] = static_cast<float>(
      std::clamp(v_new, params.min_variance, params.max_variance));
  } else {
    // Multi-height / outlier policy.
    const double age = stamp_sec - layers.timestamp_map[idx];
    const bool fresh = age >= 0.0 && age <= params.scanning_duration_sec;
    if (fresh && z_meas < z_curr) {
      // Lower point in the same scan window: likely transient occluder
      // (e.g. dynamic object). Keep the existing higher surface. The
      // timestamp/count updates at the bottom still apply: this consumed
      // a measurement, and refreshing the scan window keeps subsequent
      // same-scan low points classified as "same scan".
    } else if (fresh && z_meas > z_curr) {
      // Higher point in the same scan window: surface raised (new
      // obstacle on top). Reseed value + variance from this measurement.
      // count is intentionally not reset here so it keeps reflecting how
      // many measurements have hit this XY cell (cumulative observation
      // count rather than "samples since last reseed"); same convention
      // as MonKey-Robotics / ANYbotics.
      layers.height_map[idx]   = static_cast<float>(z_meas);
      layers.variance_map[idx] = static_cast<float>(clamped_meas);
    } else {
      // Different scan window: bump variance to admit future correction.
      const double v_bumped = v_curr + params.multi_height_noise;
      layers.variance_map[idx] = static_cast<float>(
        std::clamp(v_bumped, params.min_variance, params.max_variance));
    }
  }

  // Always refresh timestamp/count once any measurement landed in this
  // cell. Even ignored / variance-bumped measurements count as
  // "observed", and the timestamp drives the same-scan-window logic
  // above as well as prune_stale_cells's staleness gate.
  layers.timestamp_map[idx] = stamp_sec;
  layers.count_map[idx]    += 1;
  return true;
}

void fuse_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  double stamp_sec,
  const GridSpec & spec,
  const FusionParams & params,
  MapLayers & layers)
{
  for (const auto & p : cloud.points) {
    fuse_point_into_cell(p, stamp_sec, spec, params, layers);
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
