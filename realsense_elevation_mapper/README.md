# realsense_elevation_mapper

ROS2 Python node that takes a RealSense `PointCloud2` topic, accumulates the
last `k` frames into a target frame (default `base_link`), and publishes both
the raw accumulated cloud and an elevation grid (mean z per cell) as
PointCloud2 topics for RViz visualization.

Detailed design: see [../realsense_elevation_map_task_plan.md](../realsense_elevation_map_task_plan.md).

## Build

From the workspace root (`realsense_pcd_to_elev_map/`):

```bash
colcon build --packages-select realsense_elevation_mapper
source install/setup.bash
```

## Run

This node needs `target_frame` (`base_link` by default) to exist in TF.
The simplest way is to launch the bringup which also starts a state
estimator that publishes an identity `odom -> base_link` transform:

```bash
ros2 launch realsense_perception_bringup bringup.launch.py
```

Standalone (assumes some TF source is providing `<cloud frame> -> base_link`):

```bash
ros2 launch realsense_elevation_mapper local_elevation_mapper.launch.py
```

## Topics

Subscribed:

- `/camera/camera/depth/color/points` (`sensor_msgs/PointCloud2`) — RealSense.

Published:

- `/local_elevation_map/accumulated_points` (`PointCloud2`) — last `k` frames merged.
- `/local_elevation_map/points` (`PointCloud2`) — one point per occupied grid cell at the cell center, z = mean elevation.

## RViz

Set Fixed Frame to `base_link` (or `odom` if running the full bringup).
Add two PointCloud2 displays, one per output topic.
