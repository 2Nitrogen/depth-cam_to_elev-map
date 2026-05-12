#include "realsense_elevation_mapper/elevation_grid.hpp"

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

void compute_mean_elevation(
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const GridSpec & spec,
  std::vector<float> & height_map,
  std::vector<int> & count_map)
{
  const std::size_t n_cells = spec.size_x * spec.size_y;
  height_map.assign(n_cells, std::numeric_limits<float>::quiet_NaN());
  count_map.assign(n_cells, 0);

  if (cloud.empty() || n_cells == 0) {
    return;
  }

  std::vector<double> sum_map(n_cells, 0.0);
  const double inv_res = 1.0 / spec.resolution;

  for (const auto & p : cloud.points) {
    const double dx = (static_cast<double>(p.x) - spec.x_min) * inv_res;
    const double dy = (static_cast<double>(p.y) - spec.y_min) * inv_res;
    if (dx < 0.0 || dy < 0.0) {
      continue;
    }
    const std::size_t ix = static_cast<std::size_t>(dx);
    const std::size_t iy = static_cast<std::size_t>(dy);
    if (ix >= spec.size_x || iy >= spec.size_y) {
      continue;
    }
    const std::size_t idx = ix * spec.size_y + iy;
    sum_map[idx] += static_cast<double>(p.z);
    count_map[idx] += 1;
  }

  for (std::size_t i = 0; i < n_cells; ++i) {
    if (count_map[i] > 0) {
      height_map[i] = static_cast<float>(sum_map[i] / count_map[i]);
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr grid_to_points(
  const std::vector<float> & height_map,
  const std::vector<int> & count_map,
  const GridSpec & spec,
  int min_points_per_cell)
{
  auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  out->points.reserve(spec.size_x * spec.size_y / 4);  // rough upper guess

  for (std::size_t ix = 0; ix < spec.size_x; ++ix) {
    for (std::size_t iy = 0; iy < spec.size_y; ++iy) {
      const std::size_t idx = ix * spec.size_y + iy;
      if (count_map[idx] < min_points_per_cell) {
        continue;
      }
      pcl::PointXYZ p;
      p.x = static_cast<float>(spec.x_min + (static_cast<double>(ix) + 0.5) * spec.resolution);
      p.y = static_cast<float>(spec.y_min + (static_cast<double>(iy) + 0.5) * spec.resolution);
      p.z = height_map[idx];
      out->points.push_back(p);
    }
  }
  out->width = static_cast<std::uint32_t>(out->points.size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

}  // namespace realsense_elevation_mapper
