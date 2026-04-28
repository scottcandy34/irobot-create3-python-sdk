#
# Circle Arc Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with circle arcs, including finding contiguous arc segments of inliers along a circle."""

import math

def find(inliers: list[tuple[float, float]], cx: float, cy: float, r: float, max_angular_gap: float, min_points: int = 2) -> list[list[tuple[float, float]]]:
    """Group circle inliers into contiguous arc segments.

    This is the exact circle equivalent of `line_segment.find`. Points are
    sorted by angular position around the circle center, then grouped into
    arcs whenever the gap between consecutive points is ≤ max_angular_gap.

    Parameters
    ----------
    inliers : list[tuple[float, float]]
        Points that lie on (or very near) the fitted circle.
    cx, cy : float
        Center of the fitted circle.
    r : float
        Radius of the fitted circle (unused here but kept for API symmetry).
    max_angular_gap : float
        Maximum allowed angular gap (in degrees) between consecutive points
        to still consider them part of the same arc.
    min_points : int
        Minimum number of points required for an arc to be returned.

    Returns
    -------
    list[list[tuple[float, float]]]
        List of arcs. Each arc is a list of points in angular order.
        Empty list if no valid arcs are found.
    """
    if not inliers or len(inliers) < min_points:
        return []

    # Helper to get angle (degrees) from circle center
    def get_angle(p: tuple[float, float]) -> float:
        return math.degrees(math.atan2(p[1] - cy, p[0] - cx))

    # Sort points by angular position
    sorted_indices = sorted(range(len(inliers)), key=lambda i: get_angle(inliers[i]))
    sorted_points = [inliers[i] for i in sorted_indices]
    sorted_angles = [get_angle(p) for p in sorted_points]

    arcs: list[list[tuple[float, float]]] = []
    current_arc = [sorted_points[0]]

    for i in range(1, len(sorted_points)):
        angle_diff = sorted_angles[i] - sorted_angles[i - 1]
        if angle_diff <= max_angular_gap:
            current_arc.append(sorted_points[i])
        else:
            if len(current_arc) >= min_points:
                arcs.append(current_arc)
            current_arc = [sorted_points[i]]

    # Don't forget the last arc
    if len(current_arc) >= min_points:
        arcs.append(current_arc)

    return arcs
