"""sensor_msgs/Imu helpers.

Currently just unpacks an Imu message into (stamp_sec, lin_accel, ang_vel,
orientation_or_None). Future split-mode time synchronization between
accel/gyro samples can be added here without touching the node code.
"""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from sensor_msgs.msg import Imu


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def imu_msg_to_arrays(
    msg: Imu,
) -> Tuple[float, np.ndarray, np.ndarray, Optional[np.ndarray]]:
    stamp_sec = stamp_to_sec(msg.header.stamp)
    lin_accel = np.array(
        [msg.linear_acceleration.x,
         msg.linear_acceleration.y,
         msg.linear_acceleration.z],
        dtype=np.float64,
    )
    ang_vel = np.array(
        [msg.angular_velocity.x,
         msg.angular_velocity.y,
         msg.angular_velocity.z],
        dtype=np.float64,
    )

    # orientation_covariance[0] == -1 indicates the orientation field is unset
    # (sensor_msgs/Imu convention). RealSense IMU does not provide orientation,
    # so this branch is the common case.
    orientation: Optional[np.ndarray]
    if msg.orientation_covariance[0] < 0:
        orientation = None
    else:
        orientation = np.array(
            [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w],
            dtype=np.float64,
        )
    return stamp_sec, lin_accel, ang_vel, orientation
