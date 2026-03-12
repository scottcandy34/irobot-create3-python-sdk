#
# Line Segment Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with line segments, including projecting points onto a line, finding segments of inliers along a line, and calculating the length of a segment along a line."""

import math as _math

def project_point(point: tuple[float, float], m: float, b: float) -> tuple[float, float]:
    """Project a point onto the line defined by y = mx + b and return the projected point."""
    
    x, y = point
    denominator = 1 + m**2
    x_proj = (x + m * y - m * b) / denominator
    y_proj = (m * x + m**2 * y + b) / denominator
    return x_proj, y_proj

def find(inliers: list[tuple[float, float]], m: float, b: float, max_gap: int, min_points=2) -> list[tuple[float, float]]:
    """Find segments of inliers along the line defined by y = mx + b, given a maximum gap between points."""

    if not inliers:
        return []
    
    # Project inliers onto the line
    projections = [project_point(point, m, b) for point in inliers]
    
    # Sort projections along the line direction
    direction = (1, m)
    norm = _math.sqrt(1 + m**2)
    direction = (1 / norm, m / norm)
    positions = [point[0] * direction[0] + point[1] * direction[1] for point in projections]
    sorted_indices = sorted(range(len(positions)), key=lambda i: positions[i])
    sorted_points = [inliers[i] for i in sorted_indices]
    sorted_positions = [positions[i] for i in sorted_indices]
    
    segments: list[tuple[float, float]] = []
    current_segment = [sorted_points[0]]
    for i in range(1, len(sorted_points)):
        if sorted_positions[i] - sorted_positions[i-1] <= max_gap:
            current_segment.append(sorted_points[i])
        else:
            if len(current_segment) >= min_points:
                segments.append(current_segment)
            current_segment = [sorted_points[i]]
    if len(current_segment) >= min_points:
        segments.append(current_segment)
    
    return segments

def calculate_length(segment: tuple[float, float], m: float, b: float) -> float:
    """Calculate the length of a segment of points along the line defined by y = mx + b."""

    if len(segment) < 2:
        return 0.0
    projections = [project_point(point, m, b) for point in segment]
    positions = [p[0] * (1 / _math.sqrt(1 + m**2)) + p[1] * (m / _math.sqrt(1 + m**2)) for p in projections]
    length = max(positions) - min(positions)
    return length
