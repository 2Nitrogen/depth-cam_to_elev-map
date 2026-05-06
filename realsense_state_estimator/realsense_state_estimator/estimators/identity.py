"""IdentityStateEstimator — placeholder estimator.

Stores the most recent IMU sample in memory but always reports the robot at
the origin with identity orientation and zero twist. This keeps downstream
consumers (e.g. the elevation mapper's tf2 lookup) functional until a real
estimator is plugged in.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .base import Pose, StateEstimatorBase, Twist


@dataclass
class _LatestImuSample:
    stamp_sec: float = 0.0
    lin_accel: np.ndarray = field(
        default_factory=lambda: np.full(3, np.nan, dtype=np.float64)
    )
    ang_vel: np.ndarray = field(
        default_factory=lambda: np.full(3, np.nan, dtype=np.float64)
    )
    orientation: Optional[np.ndarray] = None


class IdentityStateEstimator(StateEstimatorBase):
    def __init__(self) -> None:
        self._latest = _LatestImuSample()
        self._sample_count = 0

    def predict(self, stamp_sec: float) -> None:
        return

    def update_imu(
        self,
        stamp_sec: float,
        lin_accel: np.ndarray,
        ang_vel: np.ndarray,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        self._latest = _LatestImuSample(
            stamp_sec=stamp_sec,
            lin_accel=np.asarray(lin_accel, dtype=np.float64).copy(),
            ang_vel=np.asarray(ang_vel, dtype=np.float64).copy(),
            orientation=(
                np.asarray(orientation, dtype=np.float64).copy()
                if orientation is not None else None
            ),
        )
        self._sample_count += 1

    def get_pose(self) -> Pose:
        return Pose()

    def get_twist(self) -> Twist:
        return Twist()

    def reset(self) -> None:
        self._latest = _LatestImuSample()
        self._sample_count = 0

    @property
    def latest_imu(self) -> _LatestImuSample:
        return self._latest

    @property
    def sample_count(self) -> int:
        return self._sample_count
