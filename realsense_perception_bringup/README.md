# realsense_perception_bringup

Top-level launch package: starts the IMU filter (`imu_filter_madgwick`),
the state estimator, the local elevation mapper, and RViz with a
preconfigured layout — all in one `ros2 launch` call.

The RealSense camera wrapper (`realsense2_camera`) is **not** included here.
Start it separately with whatever profile / parameters fit your hardware,
e.g.:

```bash
ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true enable_gyro:=true enable_accel:=true \
    unite_imu_method:=2
```

(`unite_imu_method:=2` produces the combined `/camera/camera/imu` topic
that Madgwick consumes. The bringup remaps `imu/data_raw` →
`/camera/camera/imu` automatically.)

## Dependencies

System-installed (Humble apt): `ros-humble-imu-filter-madgwick`. The
other dependencies are declared in `package.xml` for `rosdep`.

## Build / Run

From the workspace root:

```bash
colcon build
source install/setup.bash
ros2 launch realsense_perception_bringup bringup.launch.py
```

What this starts:

```
imu_filter_madgwick_node   # /camera/camera/imu  →  /imu/data (with orientation)
state_estimator_node       # /imu/data           →  TF odom→base_link (gravity-aligned)
local_elevation_mapper     # depth pointcloud + TF → elevation map
rviz2                      # preconfigured layout
```

### Launch arguments

| arg                    | default | meaning                                                       |
|------------------------|---------|---------------------------------------------------------------|
| `source`               | `live`  | `live` or `rosbag`. `rosbag` forces `use_sim_time:=true`.     |
| `rviz`                 | `true`  | Set false to skip RViz.                                       |
| `imu_filter`           | `true`  | Set false if `/imu/data` already exists from another source.  |
| `publish_camera_mount` | `true`  | Publish a static `base_link -> camera_link` TF from `config/camera_mount.json`. Set false if your URDF / robot_state_publisher already publishes that edge. |

```bash
ros2 launch realsense_perception_bringup bringup.launch.py rviz:=false
ros2 launch realsense_perception_bringup bringup.launch.py imu_filter:=false
ros2 launch realsense_perception_bringup bringup.launch.py publish_camera_mount:=false
```

## What you should see in RViz

- Fixed Frame: `odom` (gravity-aligned).
- TF tree with `odom -> base_link` published by `state_estimator_node`.
  Roll/pitch follow the IMU; translation stays `[0, 0, 0]`.
- `AccumulatedPoints`: the last k RealSense frames merged, transformed
  into `odom`.
- `ElevationCloud`: one box per occupied grid cell, **colored by
  intensity = sqrt(variance)**. Greener = more certain; redder = still
  uncertain.
- `ColorImage`: raw RGB stream from the camera
  (`/camera/camera/color/image_raw`), shown in a separate dockable panel so
  you can cross-check the scene visually.

If TF is missing, check `state_estimator_node` and `imu_filter_madgwick`
log output. If both clouds are empty, the RealSense wrapper isn't
producing points or the TF chain to `base_link` is broken on the camera
side. If the map tilts when you roll/pitch the robot, the IMU filter
isn't running (or its quaternion isn't reaching the estimator).
