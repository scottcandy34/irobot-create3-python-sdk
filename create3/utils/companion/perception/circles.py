#
# Circle Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with circles, including fitting a circle to a set of points and calculating the distance from a point to a circle and calculating the angle range and arc length of an arc segment."""

import math

def fit_circle(points: list[tuple[float, float]]) -> tuple[float, float, float]:
    """Fit a circle to a set of 2D points using the centroid + mean-radius method.

    This is a simple, fast, closed-form solution that mirrors the style of
    `fit_line` (geometric center + average distance). It is robust for
    noisy or roughly circular data and commonly used in robotics for
    landmark fitting or arc detection.

    Parameters
    ----------
    points : list[tuple[float, float]]
        List of (x, y) coordinates (at least 3 points required).

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

    n = len(points)

    # Centroid (geometric mean)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n

    # Mean radial distance from centroid = radius
    r = sum(math.hypot(p[0] - cx, p[1] - cy) for p in points) / n

    return cx, cy, r

def distance_to_circle(point: tuple[float, float], cx: float, cy: float, r: float) -> float:
    """Calculate the absolute radial distance error from a point to a fitted circle.

    This is the circle equivalent of `distance_to_line`. It measures how far
    a point lies from the circumference — useful for residual analysis,
    outlier rejection, or error metrics.

    Parameters
    ----------
    point : tuple[float, float]
        (x, y) coordinates of the test point.
    cx, cy : float
        Center coordinates of the fitted circle.
    r : float
        Radius of the fitted circle.

    Returns
    -------
    float
        Absolute radial distance error (|distance_from_center - r|).
    """
    x, y = point
    dist_from_center = math.hypot(x - cx, y - cy)
    return abs(dist_from_center - r)

def get_angle_range(arc_points: list[tuple[float, float]], cx: float, cy: float) -> tuple[float, float]:
    """Return the angular span (start_angle, end_angle) in degrees for an arc.

    Used when constructing _CircleArc objects or for visualization.

    Parameters
    ----------
    arc_points : list[tuple[float, float]]
        Points belonging to a single contiguous arc.
    cx, cy : float
        Center of the circle.

    Returns
    -------
    tuple[float, float]
        (start_angle, end_angle) in degrees. Returns (0.0, 0.0) for empty input.
    """
    if not arc_points:
        return 0.0, 0.0

    def get_angle(p: tuple[float, float]) -> float:
        return math.degrees(math.atan2(p[1] - cy, p[0] - cx))

    angles = [get_angle(p) for p in arc_points]
    return min(angles), max(angles)

def calculate_arc_length(arc_points: list[tuple[float, float]], cx: float, cy: float, r: float) -> float:
    """Calculate the arc length (in the same units as the radius).

    This is the circle equivalent of `calculate_length` for line segments.

    Parameters
    ----------
    arc_points : list[tuple[float, float]]
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
    