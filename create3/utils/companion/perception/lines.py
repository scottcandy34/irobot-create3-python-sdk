#
# Line Tools for iRobot Create3 - Jazzy
# =====================================================================
# Created by scottcandy34 • NumPy Optimized & Revised April 2026
#
# High-performance utilities for fitting, measuring, and projecting
# straight lines in 2D point clouds (used by WALL_DETECTION).
#
# All functions are fully vectorized with NumPy for maximum speed
# while remaining backward-compatible with plain Python lists.
# =====================================================================

import math
from typing import Sequence

import numpy as np
import numpy.typing as npt

PointCloud = Sequence[tuple[float, float]] | npt.NDArray[np.float64]


def fit_line(points: PointCloud) -> tuple[float, float]:
    """Fit a straight line y = mx + b to a set of 2D points using NumPy.

    Uses np.polyfit for robust, high-speed least-squares regression.

    Parameters
    ----------
    points : list[tuple[float, float]] | np.ndarray
        List of (x, y) points or Nx2 NumPy array.

    Returns
    -------
    tuple[float, float]
        (m, b) — slope and y-intercept of the fitted line.

    Raises
    ------
    ValueError
        If fewer than 2 points are provided or the points form a vertical line.
    """
    if len(points) < 2:
        raise ValueError("At least 2 points are required to fit a line.")

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("points must be Nx2 array or list of (x, y) tuples")

    x = pts[:, 0]
    y = pts[:, 1]

    m, b = np.polyfit(x, y, 1)

    if abs(m) > 1e8:
        raise ValueError("Vertical line detected (infinite slope).")

    return float(m), float(b)


def distance_to_line(
    point: tuple[float, float] | npt.NDArray[np.float64],
    m: float,
    b: float,
) -> float | npt.NDArray[np.float64]:
    """Calculate perpendicular distance from point(s) to the line y = mx + b.

    Fully vectorized — accepts a single point or an array of points.

    Parameters
    ----------
    point : tuple[float, float] | np.ndarray
        Single (x, y) or Nx2 array of points.
    m, b : float
        Line parameters from fit_line().

    Returns
    -------
    float | np.ndarray
        Scalar distance (single point) or array of distances.
    """
    pts = np.asarray(point, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(1, -1)

    x, y = pts[:, 0], pts[:, 1]
    dist = np.abs(y - (m * x + b)) / math.sqrt(1 + m**2)

    return float(dist[0]) if len(dist) == 1 else dist


def project_point(
    point: tuple[float, float] | npt.NDArray[np.float64],
    m: float,
    b: float,
) -> tuple[float, float] | npt.NDArray[np.float64]:
    """Project point(s) onto the infinite line defined by y = mx + b.

    This is the geometric projection used by segment grouping and
    length calculation. Fully vectorized for high performance.

    Parameters
    ----------
    point : tuple[float, float] | np.ndarray
        Single (x, y) or Nx2 array of points.
    m, b : float
        Slope and y-intercept of the line.

    Returns
    -------
    tuple[float, float] | np.ndarray
        Projected point(s) as (x_proj, y_proj).
        Returns a single tuple for one point, or Nx2 array otherwise.
    """
    pts = np.asarray(point, dtype=np.float64)
    single = pts.ndim == 1
    if single:
        pts = pts.reshape(1, -1)

    x, y = pts[:, 0], pts[:, 1]
    denom = 1.0 + m**2
    x_proj = (x + m * y - m * b) / denom
    y_proj = (m * x + m**2 * y + b) / denom

    if single:
        return float(x_proj[0]), float(y_proj[0])
    return np.column_stack((x_proj, y_proj))


def calculate_length(
    segment: PointCloud, m: float, b: float
) -> float:
    """Calculate the length of a contiguous segment of points along the fitted line.

    Projects all points onto the line and returns the distance between the
    first and last projected point (i.e., length along the line direction).

    Parameters
    ----------
    segment : list[tuple[float, float]] | np.ndarray
        Points belonging to a single contiguous line segment.
    m, b : float
        Slope and y-intercept of the fitted line.

    Returns
    -------
    float
        Length of the segment along the line (in the same units as the points).
        Returns 0.0 if fewer than 2 points are provided.
    """
    if len(segment) < 2:
        return 0.0

    projections = project_point(segment, m, b)
    if isinstance(projections, tuple):
        return 0.0

    # Unit direction vector along the line
    norm = math.sqrt(1.0 + m**2)
    direction = np.array([1.0 / norm, m / norm], dtype=np.float64)

    positions = projections @ direction
    return float(max(positions) - min(positions))