#
# Circle Tools for iRobot Create3 - Jazzy
# =====================================================================
# Created by scottcandy34 • NumPy Optimized & Revised April 2026
#
# High-performance utilities for fitting circles and working with
# circular arcs (used by COLUMN_DETECTION).
#
# All functions are fully vectorized with NumPy while remaining
# backward-compatible with plain Python lists.
# =====================================================================

import math
from typing import Sequence

import numpy as np
import numpy.typing as npt

PointCloud = Sequence[tuple[float, float]] | npt.NDArray[np.float64]


def fit_circle(points: PointCloud) -> tuple[float, float, float]:
    """Fit a circle to a set of 2D points using centroid + mean-radius method.

    Fast, closed-form solution that works very well for noisy robotics data.

    Parameters
    ----------
    points : list[tuple[float, float]] | np.ndarray
        List of (x, y) points or Nx2 NumPy array.

    Returns
    -------
    tuple[float, float, float]
        (cx, cy, r) — circle center coordinates and radius.

    Raises
    ------
    ValueError
        If fewer than 3 points are provided.
    """
    if len(points) < 3:
        raise ValueError("At least 3 points are required to fit a circle.")

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("points must be Nx2 array or list of (x, y) tuples")

    cx = float(pts[:, 0].mean())
    cy = float(pts[:, 1].mean())

    radii = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    r = float(radii.mean())

    return cx, cy, r


def distance_to_circle(
    point: tuple[float, float] | npt.NDArray[np.float64],
    cx: float,
    cy: float,
    r: float,
) -> float | npt.NDArray[np.float64]:
    """Calculate the absolute radial distance error from point(s) to a fitted circle.

    This is the circle equivalent of distance_to_line. Measures how far
    a point lies from the circumference — used heavily in RANSAC inlier tests.

    Fully vectorized for maximum performance.

    Parameters
    ----------
    point : tuple[float, float] | np.ndarray
        Single (x, y) or Nx2 array of points.
    cx, cy : float
        Center of the fitted circle.
    r : float
        Radius of the fitted circle.

    Returns
    -------
    float | np.ndarray
        Scalar distance (single point) or array of distances.
    """
    pts = np.asarray(point, dtype=np.float64)
    single = pts.ndim == 1
    if single:
        pts = pts.reshape(1, -1)

    dist_from_center = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    dist = np.abs(dist_from_center - r)

    return float(dist[0]) if single else dist


def get_angle_range(
    arc_points: PointCloud, cx: float, cy: float
) -> tuple[float, float]:
    """Return the angular span (start_angle, end_angle) in degrees for an arc.

    Used when constructing Column objects or for visualization.

    Parameters
    ----------
    arc_points : list[tuple[float, float]] | np.ndarray
        Points belonging to a single contiguous arc.
    cx, cy : float
        Center of the circle.

    Returns
    -------
    tuple[float, float]
        (start_angle, end_angle) in degrees.
        Returns (0.0, 0.0) for empty input.
    """
    if len(arc_points) == 0:
        return 0.0, 0.0

    pts = np.asarray(arc_points, dtype=np.float64)
    angles = np.degrees(np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx))
    return float(angles.min()), float(angles.max())


def calculate_arc_length(
    arc_points: PointCloud, cx: float, cy: float, r: float
) -> float:
    """Calculate the arc length (in the same units as the radius).

    This is the circle equivalent of calculate_length for line segments.

    Parameters
    ----------
    arc_points : list[tuple[float, float]] | np.ndarray
        Points belonging to a single contiguous arc.
    cx, cy : float
        Center of the circle.
    r : float
        Radius of the circle.

    Returns
    -------
    float
        Arc length. Returns 0.0 if fewer than 2 points are provided.
    """
    if len(arc_points) < 2:
        return 0.0

    start_angle, end_angle = get_angle_range(arc_points, cx, cy)
    delta_angle_rad = math.radians(end_angle - start_angle)
    return r * abs(delta_angle_rad)