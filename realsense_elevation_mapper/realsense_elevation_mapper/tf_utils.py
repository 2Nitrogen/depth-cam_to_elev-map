"""tf2 helpers for transforming Nx3 numpy point arrays between frames."""
from __future__ import annotations

import numpy as np
from geometry_msgs.msg import TransformStamped


def transform_to_matrix(t: TransformStamped) -> np.ndarray:
    """Convert TransformStamped into a 4x4 homogeneous matrix."""
    tx = t.transform.translation.x
    ty = t.transform.translation.y
    tz = t.transform.translation.z
    qx = t.transform.rotation.x
    qy = t.transform.rotation.y
    qz = t.transform.rotation.z
    qw = t.transform.rotation.w

    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    rot = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
        [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
        [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=np.float64)

    m = np.eye(4, dtype=np.float64)
    m[:3, :3] = rot
    m[:3, 3] = (tx, ty, tz)
    return m


def transform_points(points: np.ndarray, t: TransformStamped) -> np.ndarray:
    if points.size == 0:
        return points
    m = transform_to_matrix(t)
    homo = np.hstack([points, np.ones((points.shape[0], 1), dtype=points.dtype)])
    out = (m @ homo.T).T[:, :3]
    return out.astype(np.float32)
