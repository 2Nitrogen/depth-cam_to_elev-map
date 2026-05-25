# realsense_state_estimator

IMU subscription + state estimation for the RealSense perception stack.

The default estimator is `gravity_from_imu`:
- Subscribes to a Madgwick-filtered IMU topic (`/imu/data` by default —
  the bringup launch starts `imu_filter_madgwick` so `/camera/camera/imu`
  → `/imu/data` happens automatically).
- Composes the upstream quaternion `q_world_imu` with the static
  `q_imu_base` rotation (looked up via TF) to get `q_world_base`.
- Strips yaw (Madgwick has no magnetometer in our setup → yaw drifts; the
  stationary use case has no reference for absolute yaw either).
- Forces translation to `[0, 0, 0]`. Real translation will come from
  a future localization stack.
- Publishes a gravity-aligned `odom -> base_link` TF and matching
  `nav_msgs/Odometry`.

Result: when the robot body rolls/pitches in place, the published
`odom -> base_link` transform reflects that rotation, so any downstream
consumer (elevation mapper, footstep planner, ...) operating in `odom`
sees a tilt-stable world.

The original `identity` estimator (zero pose) is still available via
`estimator_type: identity` for bring-up / regression testing.

## Build / Run

The estimator is normally started by `realsense_perception_bringup`,
which also launches the Madgwick filter. To run it standalone:

```bash
# 1. Madgwick (only needed if you skip the bringup launch)
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args -r imu/data_raw:=/camera/camera/imu -p use_mag:=false \
             -p world_frame:=enu -p publish_tf:=false

# 2. State estimator
colcon build --packages-select realsense_state_estimator
source install/setup.bash
ros2 launch realsense_state_estimator state_estimator.launch.py
```

## Switching estimators

`estimator_type` is a parameter:

```yaml
# config/params.yaml
estimator_type: gravity_from_imu  # default
# estimator_type: identity        # zero pose, useful for regression
```

## Verifying

```bash
# TF should reflect roll/pitch from IMU, with translation == 0:
ros2 run tf2_ros tf2_echo odom base_link

# IMU rx rate (~200 Hz on D435i):
ros2 launch realsense_state_estimator state_estimator.launch.py \
  --ros-args -p debug_log:=true
```

Roll the robot body ±30° in place — the TF should show roll/pitch
changing while translation stays [0, 0, 0]. The first IMU sample sets
the initial orientation; the first successful TF lookup of `imu_frame -> base_frame`
seeds the static rotation (logged at INFO level).

## Adding a real estimator (sketch)

1. Create `realsense_state_estimator/estimators/<name>.py` subclassing
   [`StateEstimatorBase`](realsense_state_estimator/estimators/base.py).
2. Implement `predict`, `update_imu`, `get_pose`, `get_twist`.
3. Export it from
   [`estimators/__init__.py`](realsense_state_estimator/estimators/__init__.py).
4. Add `if name == '<name>': return <YourClass>()` to `_make_estimator`
   in [`state_estimator_node.py`](realsense_state_estimator/state_estimator_node.py).
5. Set `estimator_type: <name>` in `config/params.yaml`.

If the estimator needs a static TF (like `gravity_from_imu` needs
`imu -> base`), expose a `set_imu_to_base(q_xyzw)` method (or similar
setter). The node detects it via `hasattr` and forwards the lookup
result without changing the base class interface.

No other code in this package or in `realsense_elevation_mapper` needs
to change.
