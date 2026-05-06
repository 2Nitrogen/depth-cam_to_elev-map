"""PointCloud2 <-> numpy conversion + ROI cropping."""
from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract Nx3 float32 array of xyz from a PointCloud2 message, NaNs dropped."""
    pts = point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True
    )
    arr = np.asarray(pts.tolist(), dtype=np.float32)
    if arr.ndim == 1:
        arr = arr.reshape(-1, 3)
    return arr


def crop_roi(
    points: np.ndarray,
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    z_min: float, z_max: float,
) -> np.ndarray:
    if points.size == 0:
        return points
    m = (
        (points[:, 0] >= x_min) & (points[:, 0] < x_max) &
        (points[:, 1] >= y_min) & (points[:, 1] < y_max) &
        (points[:, 2] >= z_min) & (points[:, 2] < z_max)
    )
    return points[m]


def xyz_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Build an unorganized xyz float32 PointCloud2."""
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]

    if points.size == 0:
        points = np.zeros((0, 3), dtype=np.float32)
    points = np.ascontiguousarray(points.astype(np.float32))

    return point_cloud2.create_cloud(header, fields, points)
