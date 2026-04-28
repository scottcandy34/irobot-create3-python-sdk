#
# Line Segment Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with line segments, finding segments of inliers along a line."""

import math

from .lines import project_point

def find(inliers: list[tuple[float, float]], m: float, b: float, max_gap: float, min_points: int = 2) -> list[list[tuple[float, float]]]:
    """Group line inliers into contiguous segments along the fitted line.

    This is the exact parallel to the circle version of `find`. Points are
    projected onto the line, sorted along its direction, and grouped whenever
    the gap between consecutive projected positions is ≤ max_gap.

    Parameters
    ----------
    inliers : list[tuple[float, float]]
        Points that lie on (or very near) the fitted line.
    m : float
        Slope of the fitted line.
    b : float
        Y-intercept of the fitted line.
    max_gap : float
        Maximum allowed distance gap along the line between consecutive points
        to still consider them part of the same segment.
    min_points : int
        Minimum number of points required for a segment to be returned.

    Returns
    -------
    list[list[tuple[float, float]]]
        List of segments. Each segment is a list of original points in order
        along the line. Empty list if no valid segments are found.
    """
    if not inliers or len(inliers) < min_points:
        return []

    # Project all inliers onto the line
    projections = [project_point(point, m, b) for point in inliers]

    # Unit direction vector along the line
    norm = math.sqrt(1.0 + m**2)
    direction_x = 1.0 / norm
    direction_y = m / norm

    # Scalar position of each projected point along the line
    positions = [proj[0] * direction_x + proj[1] * direction_y for proj in projections]

    # Sort by position along the line
    sorted_indices = sorted(range(len(positions)), key=lambda i: positions[i])
    sorted_points = [inliers[i] for i in sorted_indices]
    sorted_positions = [positions[i] for i in sorted_indices]

    segments: list[list[tuple[float, float]]] = []
    current_segment = [sorted_points[0]]

    for i in range(1, len(sorted_points)):
        if sorted_positions[i] - sorted_positions[i - 1] <= max_gap:
            current_segment.append(sorted_points[i])
        else:
            if len(current_segment) >= min_points:
                segments.append(current_segment)
            current_segment = [sorted_points[i]]

    # Don't forget the last segment
    if len(current_segment) >= min_points:
        segments.append(current_segment)

    return segments