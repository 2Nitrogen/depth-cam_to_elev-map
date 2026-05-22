# realsense_perception_bringup

Top-level launch package: starts the state estimator + the local elevation
mapper together, plus RViz with a preconfigured layout.

The RealSense camera wrapper (`realsense2_camera`) is **not** included here.
Start it separately with whatever profile / parameters fit your hardware,
e.g.:

```bash
ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true enable_gyro:=true enable_accel:=true \
    unite_imu_method:=2
```

(`unite_imu_method:=2` produces the combined `/camera/camera/imu` topic
used by default. Omit or set to 0 if you prefer the split topics — then
also set `imu_mode:=split` on the state estimator.)

## Build / Run

From the workspace root:

```bash
colcon build
source install/setup.bash
ros2 launch realsense_perception_bringup bringup.launch.py
```

To suppress RViz:

```bash
ros2 launch realsense_perception_bringup bringup.launch.py rviz:=false
```

## What you should see in RViz

- Fixed Frame: `odom`.
- TF tree with `odom -> base_link` (identity) published by the state estimator placeholder.
- `AccumulatedPoints` display: the last k frames of RealSense points in `base_link`.
- `ElevationCloud` display: one box per occupied grid cell.
- `ColorImage` display: raw RGB stream from the camera
  (`/camera/camera/color/image_raw`), shown in a separate dockable panel so
  you can cross-check the scene visually.

If TF is missing, the state estimator placeholder isn't running; check its
log output. If both clouds are empty, the RealSense wrapper isn't producing
points or the TF chain to `base_link` is broken on the camera side.
