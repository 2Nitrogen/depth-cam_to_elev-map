# realsense_elevation_mapper (C++)

ROS2 C++ node that takes a RealSense `PointCloud2` topic, accumulates the
last `k` frames into a target frame (default `base_link`), and publishes both
the raw accumulated cloud and an elevation grid (mean z per cell) as
PointCloud2 topics for RViz visualization.

> **Why C++ for this package only?** The point-cloud hot path stresses
> Python's PointCloud2 codec, and the planned follow-up features (normal
> estimation, foothold scoring, voxel downsampling, adaptive resolution)
> are PCL/Eigen-native. State estimation stays in Python because the IMU
> workload is light and the abstract estimator interface is more ergonomic
> there. See [the migration plan](../../.claude/plans/realsense-misty-meteor.md)
> for full rationale.

Detailed elevation-map design: see
[../realsense_elevation_map_task_plan.md](../realsense_elevation_map_task_plan.md).

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

Needs `target_frame` (default `base_link`) reachable in TF. Easiest path
is the bringup launch (state estimator publishes identity `odom -> base_link`):

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

- `/local_elevation_map/accumulated_points` (`PointCloud2`) — last `k` frames merged.
- `/local_elevation_map/points` (`PointCloud2`) — one point per occupied grid cell at the cell center, z = mean elevation.

All topic names, frame names, k, bounds, resolution, etc. are parameters in
[config/params.yaml](config/params.yaml) and unchanged from the Python version.

## Code layout

```
include/realsense_elevation_mapper/
  elevation_grid.hpp           # GridSpec + mean elevation binning
  pointcloud_utils.hpp         # in-place ROI crop on PCL clouds
  local_elevation_mapper_node.hpp
src/
  elevation_grid.cpp
  pointcloud_utils.cpp
  local_elevation_mapper_node.cpp
  main.cpp                     # rclcpp::init + spin + shutdown
```

## RViz

Set Fixed Frame to `base_link` (or `odom` if running the full bringup).
Add two PointCloud2 displays, one per output topic. The
[realsense_perception_bringup](../realsense_perception_bringup) package
provides a preconfigured `default.rviz`.
