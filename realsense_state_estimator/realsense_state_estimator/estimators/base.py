"""StateEstimatorBase: abstract interface that real estimators (Madgwick, EKF,
VIO bridge, ...) implement. Pure Python — no ROS imports here so the same
class can be unit-tested without a ROS context.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class Pose:
    position: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    # Quaternion in (x, y, z, w) order. Identity = (0, 0, 0, 1).
    orientation: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    )


@dataclass
class Twist:
    linear: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    angular: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )


class StateEstimatorBase(ABC):
    """Estimator interface used by state_estimator_node.

    Lifecycle per tick:
      1. zero or more update_*() calls fed by IMU subscriptions
      2. predict(now) called by the publish timer
      3. get_pose() / get_twist() read out for publishing
    """

    @abstractmethod
    def predict(self, stamp_sec: float) -> None:
        """Time-update step (motion model). Called by publish timer."""

    @abstractmethod
    def update_imu(
        self,
        stamp_sec: float,
        lin_accel: np.ndarray,
        ang_vel: np.ndarray,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """Combined IMU sample. orientation is None when unavailable."""

    def update_accel(self, stamp_sec: float, lin_accel: np.ndarray) -> None:
        """Split-mode accelerometer-only sample. Default: forward to update_imu
        with NaN angular velocity. Override for true async fusion."""
        nan_ang = np.full(3, np.nan, dtype=np.float64)
        self.update_imu(stamp_sec, lin_accel, nan_ang, orientation=None)

    def update_gyro(self, stamp_sec: float, ang_vel: np.ndarray) -> None:
        """Split-mode gyroscope-only sample. Default: forward to update_imu
        with NaN linear acceleration. Override for true async fusion."""
        nan_acc = np.full(3, np.nan, dtype=np.float64)
        self.update_imu(stamp_sec, nan_acc, ang_vel, orientation=None)

    @abstractmethod
    def get_pose(self) -> Pose:
        """Latest estimated pose in odom frame."""

    @abstractmethod
    def get_twist(self) -> Twist:
        """Latest estimated body-frame twist."""

    def reset(self) -> None:
        """Reset internal state. Default: no-op."""
        return
