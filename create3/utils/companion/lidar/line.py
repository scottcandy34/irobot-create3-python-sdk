#
# Line Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with lines, including fitting a line to a set of points, calculating the distance from a point to a line, and finding segments of inliers along a line."""

import math as _math

def fit_line(points: list[tuple[float, float]]) -> tuple[float, float]:
    """Fit a line to a set of points and return the slope and y-intercept."""

    x = [p[0] for p in points]
    y = [p[1] for p in points]
    n = len(points)
    sum_x = sum(x)
    sum_y = sum(y)
    sum_xy = sum(xi * yi for xi, yi in zip(x, y))
    sum_xx = sum(xi * xi for xi in x)
    denominator = n * sum_xx - sum_x**2
    if abs(denominator) < 1e-10:
        raise ValueError("Vertical line detected")
    m = (n * sum_xy - sum_x * sum_y) / denominator
    b = (sum_y - m * sum_x) / n
    return m, b

def distance_to_line(point: tuple[float, float], m: float, b: float):
    """Calculate the distance from a point to the line defined by y = mx + b."""
    
    x, y = point
    return abs(y - (m * x + b)) / _math.sqrt(1 + m**2)