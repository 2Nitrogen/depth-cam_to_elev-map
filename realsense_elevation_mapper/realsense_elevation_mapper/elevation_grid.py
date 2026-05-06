"""2D elevation grid: bin points into cells, compute mean elevation."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class GridSpec:
    x_min: float
    y_min: float
    resolution: float
    size_x: int
    size_y: int


def make_grid_spec(
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    resolution: float,
) -> GridSpec:
    size_x = int(np.floor((x_max - x_min) / resolution))
    size_y = int(np.floor((y_max - y_min) / resolution))
    return GridSpec(
        x_min=x_min, y_min=y_min,
        resolution=resolution,
        size_x=size_x, size_y=size_y,
    )


def compute_mean_elevation(points: np.ndarray, spec: GridSpec):
    """Return (height_map (size_x, size_y), count_map (size_x, size_y)).

    Cells with count==0 hold NaN in height_map.
    """
    height_map = np.full((spec.size_x, spec.size_y), np.nan, dtype=np.float32)
    count_map = np.zeros((spec.size_x, spec.size_y), dtype=np.int32)

    if points.size == 0:
        return height_map, count_map

    ix = np.floor((points[:, 0] - spec.x_min) / spec.resolution).astype(np.int64)
    iy = np.floor((points[:, 1] - spec.y_min) / spec.resolution).astype(np.int64)
    z = points[:, 2]

    in_bounds = (ix >= 0) & (ix < spec.size_x) & (iy >= 0) & (iy < spec.size_y)
    ix, iy, z = ix[in_bounds], iy[in_bounds], z[in_bounds]

    flat_idx = ix * spec.size_y + iy
    n_cells = spec.size_x * spec.size_y

    sum_flat = np.bincount(flat_idx, weights=z, minlength=n_cells).astype(np.float32)
    cnt_flat = np.bincount(flat_idx, minlength=n_cells).astype(np.int32)

    sum_map = sum_flat.reshape(spec.size_x, spec.size_y)
    count_map = cnt_flat.reshape(spec.size_x, spec.size_y)

    valid = count_map > 0
    height_map[valid] = sum_map[valid] / count_map[valid]
    return height_map, count_map


def grid_to_points(height_map: np.ndarray, count_map: np.ndarray,
                   spec: GridSpec, min_points_per_cell: int = 1) -> np.ndarray:
    """Convert valid grid cells into Nx3 (x, y, z) point cloud at cell centers."""
    valid = count_map >= min_points_per_cell
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32)

    ix, iy = np.where(valid)
    cx = spec.x_min + (ix + 0.5) * spec.resolution
    cy = spec.y_min + (iy + 0.5) * spec.resolution
    cz = height_map[ix, iy]
    return np.stack([cx, cy, cz], axis=1).astype(np.float32)
