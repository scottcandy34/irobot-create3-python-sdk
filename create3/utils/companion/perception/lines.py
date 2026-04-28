#
# Line Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with lines, including fitting a line to a set of points, calculating the distance from a point to a line, including projecting points onto a line, and calculating the length of a segment along a line."""

import math

def fit_line(points: list[tuple[float, float]]) -> tuple[float, float]:
    """Fit a straight line to a set of 2D points using least-squares regression.

    Returns the slope (m) and y-intercept (b) of the best-fit line y = mx + b.
    This is the exact parallel to `fit_circle` (closed-form, simple, and robust
    for noisy data). Commonly used in robotics for wall/line detection.

    Parameters
    ----------
    points : list[tuple[float, float]]
        List of (x, y) coordinates (at least 2 points required).

    Returns
    -------
    tuple[float, float]
        (m, b) — slope and y-intercept of the fitted line.

    Raises
    ------
    ValueError
        If fewer than 2 points are provided or the points form a vertical line
        (infinite slope).
    """
    if len(points) < 2:
        raise ValueError("At least 2 points are required to fit a line.")

    x = [p[0] for p in points]
    y = [p[1] for p in points]
    n = len(points)
    sum_x = sum(x)
    sum_y = sum(y)
    sum_xy = sum(xi * yi for xi, yi in zip(x, y))
    sum_xx = sum(xi * xi for xi in x)

    denominator = n * sum_xx - sum_x**2
    if abs(denominator) < 1e-10:
        raise ValueError("Vertical line detected (infinite slope).")

    m = (n * sum_xy - sum_x * sum_y) / denominator
    b = (sum_y - m * sum_x) / n

    return m, b

def distance_to_line(point: tuple[float, float], m: float, b: float) -> float:
    """Calculate the perpendicular distance from a point to the line y = mx + b.

    This is the line equivalent of `distance_to_circle`. It measures the
    shortest (orthogonal) distance to the fitted line — useful for residual
    analysis, inlier counting, or RANSAC-style outlier rejection.

    Parameters
    ----------
    point : tuple[float, float]
        (x, y) coordinates of the test point.
    m : float
        Slope of the fitted line.
    b : float
        Y-intercept of the fitted line.

    Returns
    -------
    float
        Perpendicular distance from the point to the line.
    """
    x, y = point
    return abs(y - (m * x + b)) / math.sqrt(1 + m**2)

def project_point(point: tuple[float, float], m: float, b: float) -> tuple[float, float]:
    """Project a point onto the line defined by y = mx + b.

    This returns the closest point on the infinite line to the given point.
    It is the geometric projection used by `find` and `calculate_length`.

    Parameters
    ----------
    point : tuple[float, float]
        (x, y) coordinates of the point to project.
    m : float
        Slope of the line.
    b : float
        Y-intercept of the line.

    Returns
    -------
    tuple[float, float]
        (x_proj, y_proj) — the projected point on the line.
    """
    x, y = point
    denominator = 1.0 + m**2
    x_proj = (x + m * y - m * b) / denominator
    y_proj = (m * x + m**2 * y + b) / denominator
    return x_proj, y_proj

def calculate_length(segment: list[tuple[float, float]], m: float, b: float) -> float:
    """Calculate the length of a contiguous segment of points along the fitted line.

    This is the line equivalent of `calculate_arc_length`. It projects the points
    onto the line and returns the distance between the first and last projected
    point (i.e., the length along the line).

    Parameters
    ----------
    segment : list[tuple[float, float]]
        Points belonging to a single contiguous segment.
    m : float
        Slope of the fitted line.
    b : float
        Y-intercept of the fitted line.

    Returns
    -------
    float
        Length of the segment along the line. Returns 0.0 if fewer than 2 points.
    """
    if len(segment) < 2:
        return 0.0

    projections = [project_point(point, m, b) for point in segment]

    # Unit direction vector along the line
    norm = math.sqrt(1.0 + m**2)
    direction_x = 1.0 / norm
    direction_y = m / norm

    positions = [p[0] * direction_x + p[1] * direction_y for p in projections]

    return max(positions) - min(positions)