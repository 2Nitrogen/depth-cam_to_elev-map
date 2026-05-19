# realsense_pcd_to_elev_map

This repository is a ROS2 perception software stack that builds a local
elevation map from the PointCloud2 / IMU topics published by a RealSense
depth camera. The repository root doubles as the `src/` directory of a
colcon workspace.

Detailed design documents:
- Overall elevation mapper design: [realsense_elevation_map_task_plan.md](realsense_elevation_map_task_plan.md)
- IMU / state estimation design: `~/.claude/plans/realsense-misty-meteor.md`

---

## Packages

```
realsense_pcd_to_elev_map/                  (colcon workspace src)
├─ realsense_elevation_mapper/              (1) PointCloud2 → elevation grid
├─ realsense_state_estimator/               (2) IMU subscription + state estimation skeleton
└─ realsense_perception_bringup/            (3) combined bring-up launch + RViz config
```

### (1) `realsense_elevation_mapper` (C++)

Subscribes to `PointCloud2`, accumulates the latest k frames, bins them
into a 2D grid, and publishes the result as PointCloud2. Implemented in
**C++ (rclcpp + PCL + Eigen)** — this is the hot path of the stack and
also where the planned follow-on features (normal estimation, foothold
scoring, voxel downsampling, ...) naturally live in the PCL/Eigen
ecosystem. See
[realsense_elevation_mapper/README.md](realsense_elevation_mapper/README.md)
for details.

Core files:
- [src/main.cpp](realsense_elevation_mapper/src/main.cpp) — Entry point (`rclcpp::init` + spin)
- [src/local_elevation_mapper_node.cpp](realsense_elevation_mapper/src/local_elevation_mapper_node.cpp) — Node implementation (subs/pubs/TF)
- [src/elevation_grid.cpp](realsense_elevation_mapper/src/elevation_grid.cpp) — Grid binning + mean elevation
- [src/pointcloud_utils.cpp](realsense_elevation_mapper/src/pointcloud_utils.cpp) — ROI crop on PCL clouds
- [include/realsense_elevation_mapper/](realsense_elevation_mapper/include/realsense_elevation_mapper/) — Public headers
- [CMakeLists.txt](realsense_elevation_mapper/CMakeLists.txt) — Build configuration
- [config/params.yaml](realsense_elevation_mapper/config/params.yaml) — Parameters

TF application is handled via `tf2_eigen` + `pcl::transformPointCloud`, so
no equivalent of the old `tf_utils.py` is needed.

Extra system dependencies for this package (beyond a ROS2 desktop install):
`ros-humble-pcl-conversions`, `ros-humble-tf2-eigen`,
`ros-humble-tf2-sensor-msgs`, `libpcl-dev`, `libeigen3-dev`. With `rosdep`
installed, `rosdep install --from-paths . --ignore-src -y` at the
workspace root pulls these in automatically.

### (2) `realsense_state_estimator`

Subscribes to the RealSense IMU topic and publishes TF
(`odom -> base_link`) + `nav_msgs/Odometry` through the
`StateEstimatorBase` interface. **Currently a placeholder
(IdentityStateEstimator).** See
[realsense_state_estimator/README.md](realsense_state_estimator/README.md)
for details.

Core files:
- [state_estimator_node.py](realsense_state_estimator/realsense_state_estimator/state_estimator_node.py) — Core node (subs/pubs/timer)
- [estimators/base.py](realsense_state_estimator/realsense_state_estimator/estimators/base.py) — `StateEstimatorBase` interface + `Pose`/`Twist` dataclass
- [estimators/identity.py](realsense_state_estimator/realsense_state_estimator/estimators/identity.py) — Placeholder
- [imu_utils.py](realsense_state_estimator/realsense_state_estimator/imu_utils.py) — `sensor_msgs/Imu` → numpy
- [config/params.yaml](realsense_state_estimator/config/params.yaml) — IMU mode, frame, rate

### (3) `realsense_perception_bringup`

A combined launch that brings up (1) and (2) together with a preset RViz
config. It does not include the RealSense camera wrapper itself; that is
expected to be running separately. See
[realsense_perception_bringup/README.md](realsense_perception_bringup/README.md)
for details.

---

## Data Flow

```
RealSense ROS2 wrapper
   ├─ /camera/.../points  ──►  realsense_elevation_mapper
   └─ /camera/.../imu     ──►  realsense_state_estimator
                                     │
                       publishes:    ▼
                 TF: odom → base_link
                 /state_estimator/odometry
                                     │
   realsense_elevation_mapper  ◄── tf2 lookup ──┘
        │
        ├─ /local_elevation_map/accumulated_points
        └─ /local_elevation_map/points
                  │
                  ▼
                 RViz
```

The two nodes are **coupled only through TF**. Even when the estimator is
later swapped out for an EKF/VIO implementation, the elevation mapper
code does not have to change.

---

## Quick start

```bash
cd /home/sequor/realsense_pcd_to_elev_map
colcon build
source install/setup.bash

# 1. RealSense camera (separate terminal)
ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2

# 2. Perception stack + RViz
ros2 launch realsense_perception_bringup bringup.launch.py
```

> **TF note**: a static transform connecting the RealSense camera frame to
> `base_link` is required. On an actual robot, the URDF /
> robot_state_publisher provides it. For a rosbag-replay workflow without
> a URDF, launch with `source:=rosbag` — the bringup then publishes that
> static TF itself, reading translation / RPY from
> [`realsense_perception_bringup/config/camera_mount.json`](realsense_perception_bringup/config/camera_mount.json).
> The shipped defaults are all zeros (identity), which is fine for a
> desktop demo; edit the file (or override via
> `camera_mount_config:=/path/to/custom.json`) with the actual mount
> values for your robot.

---

## Caveats when replaying rosbags

When data is replayed through `ros2 bag play` instead of a live camera,
there are two time / TF traps. Both are obvious once you've hit them but
tricky to diagnose the first time.

### 1. Aligning the time domain: `--clock` + `use_sim_time:=true`

Bag messages carry the timestamps from when they were recorded, while the
TF / Odometry published by the nodes is stamped with the **current** time.
Replay them naively and the two time axes are completely disjoint, so TF
lookups always fail (hour-scale gaps).

Correct invocation:

```bash
# Terminal A
ros2 bag play <bag_path> --clock 100

# Terminal B
ros2 launch realsense_perception_bringup bringup.launch.py source:=rosbag
```

`source:=rosbag` forces `use_sim_time:=true` for every node in the
bringup, so you do not need to pass it separately.

### 2. RealSense sensor-time vs. record-time mismatch

Even with `--clock` + `use_sim_time:=true`, a **constant offset of a few
seconds** can remain between the cloud's `header.stamp` and `/clock`. This
is because the RealSense ROS2 wrapper stamps messages with the
**sensor hardware clock** by default, while `--clock` publishes the bag's
record time (system clock). The two clocks are not aligned.

**Root fix**: re-record the bag with `use_ros_time:=true` so the wrapper
stamps messages in system time:
```bash
ros2 launch realsense2_camera rs_launch.py ... use_ros_time:=true
```

**Prototyping with an already-recorded bag**: leave the elevation_mapper
parameter `use_latest_tf` at true (the default). It ignores the cloud's
`header.stamp` and looks up the latest available TF instead, sidestepping
the offset. Switch it back to false once a time-varying estimator
(Madgwick/EKF, ...) is in place, since the latest-TF mode becomes
inaccurate then.

### 3. Check for a disconnected TF tree

A bag typically only contains `camera_link → camera_*_frame`, and the
state_estimator only publishes `odom → base_link`. If nothing connects
`base_link ↔ camera_link`, the tree is split in two.

Check:
```bash
ros2 run tf2_tools view_frames    # generates a tree PDF
# or
ros2 run tf2_ros tf2_echo base_link camera_link
```

Launching with `source:=rosbag` already handles this — the bringup
spawns a `static_transform_publisher` that publishes the
`base_link → camera_link` transform from
[`config/camera_mount.json`](realsense_perception_bringup/config/camera_mount.json).
If the tree is still split (e.g. you used `source:=live` for a bag
replay), switch to `source:=rosbag`, or edit the JSON / point
`camera_mount_config:=` at your own file with the correct mount values.

---

## Guide for adding future features

### A. Adding a real state estimator (Madgwick / complementary / EKF / VIO)

The structure lets you add a single class without touching the ROS node code.

1. Create `realsense_state_estimator/realsense_state_estimator/estimators/<name>.py`
   inheriting from
   [`StateEstimatorBase`](realsense_state_estimator/realsense_state_estimator/estimators/base.py):
   ```python
   class MadgwickEstimator(StateEstimatorBase):
       def __init__(self, beta: float = 0.1):
           ...
       def predict(self, stamp_sec): ...
       def update_imu(self, stamp_sec, lin_accel, ang_vel, orientation=None): ...
       def get_pose(self) -> Pose: ...
       def get_twist(self) -> Twist: ...
   ```
2. Export it from
   [`estimators/__init__.py`](realsense_state_estimator/realsense_state_estimator/estimators/__init__.py).
3. Add a single line to `_make_estimator` in
   [`state_estimator_node.py`](realsense_state_estimator/realsense_state_estimator/state_estimator_node.py):
   ```python
   if name == 'madgwick':
       return MadgwickEstimator(beta=...)
   ```
   Expose any extra parameters via `declare_parameter` and pass them into
   `_make_estimator`.
4. Switch
   [`config/params.yaml`](realsense_state_estimator/config/params.yaml) to
   `estimator_type: madgwick`.
5. Verify that `ros2 topic echo /tf` / `/state_estimator/odometry` shows
   sensible values.

If you need split-mode asynchronous fusion, override `update_accel` /
`update_gyro` directly (the default implementation forwards to `update_imu`
with NaN-filled fields).

### B. Adding elevation map output formats (`grid_map_msgs`, `OccupancyGrid`, markers, ...)

PointCloud2 is the only output today. Adding more is a small step (C++).

1. Add the output topic name and an enable flag to
   [`config/params.yaml`](realsense_elevation_mapper/config/params.yaml).
2. In
   [`src/local_elevation_mapper_node.cpp`](realsense_elevation_mapper/src/local_elevation_mapper_node.cpp):
   - Add a new `rclcpp::Publisher<...>::SharedPtr` member and create it in the constructor.
   - Feed the `height_map` / `count_map` filled by `compute_mean_elevation`
     into a new conversion function that produces the new message, then publish.
3. Declare the conversion helper in
   [`include/realsense_elevation_mapper/elevation_grid.hpp`](realsense_elevation_mapper/include/realsense_elevation_mapper/elevation_grid.hpp)
   and implement it in
   [`src/elevation_grid.cpp`](realsense_elevation_mapper/src/elevation_grid.cpp)
   (e.g. `nav_msgs::msg::OccupancyGrid grid_to_occupancy_grid(...)`).
4. Add the new message-package dependency to **both**
   [`package.xml`](realsense_elevation_mapper/package.xml) (`<depend>...</depend>`)
   and
   [`CMakeLists.txt`](realsense_elevation_mapper/CMakeLists.txt)
   (`find_package(...)` + `ament_target_dependencies(...)`).

### C. Extending cell statistics (variance, median, min/max)

1. Add `compute_*_elevation` sibling functions to
   [`include/realsense_elevation_mapper/elevation_grid.hpp`](realsense_elevation_mapper/include/realsense_elevation_mapper/elevation_grid.hpp)
   +
   [`src/elevation_grid.cpp`](realsense_elevation_mapper/src/elevation_grid.cpp).
2. Branch on the chosen statistic via a parameter
   (`elevation_estimator: mean | median | ...`).
3. Unit tests are easy to add: the `elevation_grid` module has no ROS
   dependency, so a GoogleTest target under `test/` plus `ament_add_gtest`
   in [`CMakeLists.txt`](realsense_elevation_mapper/CMakeLists.txt) is
   enough.

### D. Adding a new package (e.g. foothold scoring)

The recommended pattern is to add it as a sibling package. Pick a language
based on the workload:

- **Python** for light orchestration / IMU-rate / abstract-interface-driven
  logic — copy `realsense_state_estimator/` as a template, then write
  `package.xml`, `setup.py`, `setup.cfg`, `resource/<new_pkg>` with names
  swapped.
- **C++** for point-cloud / PCL-Eigen-heavy work — copy
  `realsense_elevation_mapper/` as a template, then write `package.xml`
  (build_type=`ament_cmake`) and `CMakeLists.txt` with names swapped.

Then:

1. Create `realsense_pcd_to_elev_map/<new_pkg>/` and fill it from the
   chosen template.
2. Subscribe to `/local_elevation_map/points` (or whatever grid_map topic
   you have added); publish whatever you need — markers, point clouds,
   custom topics.
3. Add an `IncludeLaunchDescription` for it in
   [`bringup.launch.py`](realsense_perception_bringup/launch/bringup.launch.py).
4. Add the package name to `<exec_depend>` in
   [`bringup/package.xml`](realsense_perception_bringup/package.xml).
5. Add a display entry in the RViz config
   ([default.rviz](realsense_perception_bringup/rviz/default.rviz)).

### E. Pure parameter tuning

No code changes required — yaml or CLI is enough.

```bash
# One-off (runtime override)
ros2 launch realsense_elevation_mapper local_elevation_mapper.launch.py \
    --ros-args -p resolution:=0.01 -p k_frames:=10

# Permanent
# Edit the package's config/params.yaml, then colcon build (the file is copied into install/share).
```

To inject an external yaml wholesale via a launch argument:

```bash
ros2 launch realsense_perception_bringup bringup.launch.py \
    params_file:=/path/to/custom.yaml
```

(The bringup currently uses each package's default params. To support
external yaml injection, add `launch_arguments={'params_file': ...}` to the
`IncludeLaunchDescription` in
[`bringup.launch.py`](realsense_perception_bringup/launch/bringup.launch.py).)

---

## Directory / file conventions

- Package names keep the `realsense_*` prefix.
- Separate ROS-independent code (C++: `elevation_grid.cpp` /
  `pointcloud_utils.cpp`; Python: `estimators/base.py`, `imu_utils.py`)
  from ROS node code (`*_node.cpp` / `*_node.py`). The former is
  unit-test friendly.
- Language-by-package rule of thumb: **C++** for hot paths and
  PCL/Eigen-heavy work (currently `realsense_elevation_mapper`),
  **Python** for orchestration and abstract-interface-driven logic
  (currently `realsense_state_estimator`; launch-only packages like
  `realsense_perception_bringup` use `ament_cmake` for install rules only).
  Apply the same rule when adding new packages.
- Every parameter lives in yaml; nodes load them via
  `declare_parameter` + `get_parameter` (same API in rclcpp and rclpy).
- When adding a new estimator / output format, **add a new file** and limit
  the node body change to a single line. If you find yourself frequently
  editing the node body, the interface design is wrong.

---

## Development workflow

```bash
# After a code change
colcon build --packages-select <changed_package>
source install/setup.bash
ros2 launch ...
```

When the interface (skeleton) is unchanged, only the modified packages
need to be rebuilt — that's fast. When the interface changes, rebuild
every dependent package as well.

C++ rebuild iteration: the first `realsense_elevation_mapper` build is
slow (~30–60 s, PCL headers); incremental rebuilds of just this package
are fast.

Python iteration is faster with `--symlink-install` — `.py` files
modified after build take effect on the next launch with no rebuild:

```bash
colcon build --packages-select realsense_state_estimator --symlink-install
```

Pure Python modules can be unit-tested without ROS:

```bash
cd realsense_state_estimator
python3 -c "
from realsense_state_estimator.estimators.identity import IdentityStateEstimator
import numpy as np
e = IdentityStateEstimator()
e.update_imu(0.0, np.zeros(3), np.zeros(3))
print(e.get_pose())
"
```
