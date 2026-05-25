# realsense_elevation_mapper (C++)

ROS2 C++ node that takes a RealSense `PointCloud2` topic, accumulates the
last `k` frames into a target frame (default `odom`, gravity-aligned), and
publishes both the raw accumulated cloud and a 1-D Kalman-fused elevation
grid as PointCloud2 topics for RViz visualization.

Per-cell update model (see [include/realsense_elevation_mapper/elevation_grid.hpp](include/realsense_elevation_mapper/elevation_grid.hpp)):
empty cell → initialize from the measurement; existing cell → Mahalanobis
gate, then either 1-D Kalman fusion or a multi-height/outlier policy
(replace-if-higher, ignore-if-lower within the same scan window;
variance bump otherwise). Each cell carries (elevation, variance,
timestamp, count). Output points are `PointXYZI` with `intensity = sqrt(variance)`
so RViz colors cells by their 1σ uncertainty.

> **Why C++ for this package only?** The point-cloud hot path stresses
> Python's PointCloud2 codec, and the planned follow-up features (normal
> estimation, foothold scoring, voxel downsampling, adaptive resolution)
> are PCL/Eigen-native. State estimation stays in Python because the IMU
> workload is light and the abstract estimator interface is more
> ergonomic there.

## System dependencies

Ubuntu 22.04 + ROS2 humble desktop install usually has everything. If not:

```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions \
                 ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs \
                 libpcl-dev libeigen3-dev
```

## Build

From the workspace root (`realsense_pcd_to_elev_map/`):

```bash
colcon build --packages-select realsense_elevation_mapper
source install/setup.bash
```

The first build is slow (~30–60 s) because of PCL. Subsequent rebuilds of
just this package are fast.

## Run

Needs `target_frame` (default `odom`) reachable in TF. The bringup launch
sets up the full chain (RealSense → Madgwick → state estimator → mapper):

```bash
ros2 launch realsense_perception_bringup bringup.launch.py
```

Standalone:

```bash
ros2 launch realsense_elevation_mapper local_elevation_mapper.launch.py
```

## Topics

Subscribed:

- `/camera/camera/depth/color/points` (`sensor_msgs/PointCloud2`) — RealSense.

Published:

- `/local_elevation_map/accumulated_points` (`PointCloud2`, XYZ) — last `k`
  frames merged, in `target_frame`.
- `/local_elevation_map/points` (`PointCloud2`, XYZI) — one point per
  occupied grid cell at the cell center. `z` = Kalman-fused elevation,
  `intensity` = sqrt(variance) (i.e. 1σ uncertainty in meters).

All topic names, frame names, k, bounds, resolution, and fusion knobs are
parameters in [config/params.yaml](config/params.yaml).

## Frames

- `target_frame` (default `odom`) — gravity-aligned global frame the map
  lives in. Elevation z is world-up.
- `track_point_frame` (default `base_link`) — body frame the local map
  window follows. When ROI bounds are set in YAML, they are interpreted
  as offsets relative to this frame's XY position in `target_frame`.

## Code layout

```
include/realsense_elevation_mapper/
  elevation_grid.hpp           # GridSpec, MapLayers, FusionParams, Kalman fuse
  pointcloud_utils.hpp         # in-place ROI crop on PCL clouds
  local_elevation_mapper_node.hpp
src/
  elevation_grid.cpp
  pointcloud_utils.cpp
  local_elevation_mapper_node.cpp
  main.cpp                     # rclcpp::init + spin + shutdown
```

## RViz

Set Fixed Frame to `odom`. The
[realsense_perception_bringup](../realsense_perception_bringup) package
provides a preconfigured `default.rviz` that colors `ElevationCloud`
by intensity (low = green, high = red), so RViz visually shows where the
estimate is still uncertain.
