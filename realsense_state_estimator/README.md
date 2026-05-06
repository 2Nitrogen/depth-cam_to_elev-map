# realsense_state_estimator

IMU subscription + state estimation **skeleton** for the RealSense
perception stack.

The current implementation is intentionally a placeholder:

- Subscribes to RealSense IMU (combined `/camera/camera/imu` by default,
  or split `/accel/sample` + `/gyro/sample`).
- Stores the latest IMU sample in memory.
- Publishes an **identity** `odom -> base_link` TF and an identity
  `nav_msgs/Odometry` so downstream consumers (e.g. the elevation mapper's
  tf2 lookup) work end-to-end without a real estimator.

Real estimators (Madgwick, complementary filter, EKF, VIO bridge) plug in
by subclassing
[`StateEstimatorBase`](realsense_state_estimator/estimators/base.py) and
extending `_make_estimator` in
[`state_estimator_node.py`](realsense_state_estimator/state_estimator_node.py).
The ROS node code does not change when an estimator is swapped.

## Build / Run

```bash
colcon build --packages-select realsense_state_estimator
source install/setup.bash
ros2 launch realsense_state_estimator state_estimator.launch.py
```

## Switching IMU mode

`imu_mode` is a parameter; default is `combined`. Switching to `split`
requires the RealSense wrapper to publish `/accel/sample` and `/gyro/sample`
(this is the wrapper's default when `unite_imu_method` is not set):

```bash
ros2 launch realsense_state_estimator state_estimator.launch.py \
  --ros-args -p imu_mode:=split
```

## Verifying placeholder behavior

```bash
ros2 topic echo /tf | head            # odom -> base_link identity at 50 Hz
ros2 topic echo /state_estimator/odometry
ros2 launch realsense_state_estimator state_estimator.launch.py \
  --ros-args -p debug_log:=true       # logs IMU rx rate ~1 Hz
```

## Adding a real estimator (sketch)

1. Create `realsense_state_estimator/estimators/madgwick.py` subclassing
   `StateEstimatorBase`.
2. Implement `predict`, `update_imu`, `get_pose`, `get_twist`.
3. Add `if name == 'madgwick': return MadgwickEstimator(...)` to
   `_make_estimator`.
4. Set `estimator_type: madgwick` in `config/params.yaml`.

No other code in this package or in `realsense_elevation_mapper` needs
to change.
