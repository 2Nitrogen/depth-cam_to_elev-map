"""GravityFromImuQuaternion — gravity-aligned attitude from a Madgwick (or
similar) IMU orientation quaternion.

Pipeline:
  * Upstream `imu_filter_madgwick_node` consumes raw accel + gyro and emits
    a sensor_msgs/Imu whose `.orientation` is q_world_imu (rotation of the
    IMU frame in a gravity-aligned world frame, ENU by default).
  * This estimator composes that with the static `q_imu_base` (looked up
    once by the node via TF) to obtain q_world_base.
  * Yaw is then forced to zero: without a magnetometer Madgwick's yaw is
    free to drift, and the stationary-robot use case has no reference for
    absolute yaw. Roll and pitch — the gravity-observable components — are
    preserved.

The resulting `Pose` has position [0, 0, 0] (translation is intentionally
zero until odometry is wired in) and orientation q_world_base with yaw
removed.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .base import Pose, StateEstimatorBase, Twist


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product (xyzw) — a * b composes rotations 'b then a'."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=np.float64)


def _quat_yaw_zero(q: np.ndarray) -> np.ndarray:
    """Strip the yaw component (intrinsic ZYX, ROS convention).

    Returns a quaternion encoding the same roll and pitch but yaw = 0.
    """
    x, y, z, w = q
    # Roll (x-axis): atan2(2(wx + yz), 1 - 2(x^2 + y^2))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # Pitch (y-axis): asin(2(wy - zx)), clamped for gimbal lock.
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # Reconstruct with yaw = 0:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    return np.array([
        sr * cp,         # x
        cr * sp,         # y
        -sr * sp,        # z
        cr * cp,         # w
    ], dtype=np.float64)


class GravityFromImuQuaternion(StateEstimatorBase):
    """Pose: position = [0,0,0], orientation = roll/pitch from IMU quaternion."""

    def __init__(self) -> None:
        self._q_world_imu: Optional[np.ndarray] = None
        self._q_imu_base: Optional[np.ndarray] = None

    def set_imu_to_base(self, q_imu_base: np.ndarray) -> None:
        """Inject the static IMU-to-base rotation (looked up via TF by the node).

        q_imu_base rotates a vector from base coordinates into IMU coordinates,
        i.e. v_imu = R(q_imu_base) * v_base.
        """
        q = np.asarray(q_imu_base, dtype=np.float64).copy()
        norm = float(np.linalg.norm(q))
        if norm > 0.0:
            q /= norm
        self._q_imu_base = q

    def predict(self, stamp_sec: float) -> None:
        return

    def update_imu(
        self,
        stamp_sec: float,
        lin_accel: np.ndarray,
        ang_vel: np.ndarray,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        if orientation is None:
            return
        q = np.asarray(orientation, dtype=np.float64)
        norm = float(np.linalg.norm(q))
        if norm == 0.0:
            return
        self._q_world_imu = q / norm

    def get_pose(self) -> Pose:
        pose = Pose()  # position = zeros, orientation = identity
        if self._q_world_imu is None or self._q_imu_base is None:
            return pose
        q_world_base = _quat_multiply(self._q_world_imu, self._q_imu_base)
        pose.orientation = _quat_yaw_zero(q_world_base)
        return pose

    def get_twist(self) -> Twist:
        return Twist()

    def reset(self) -> None:
        self._q_world_imu = None
