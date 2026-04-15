import math

def find(inliers: list[tuple[float, float]], cx: float, cy: float, r: float, max_angular_gap: float, min_points=2) -> list[list[tuple[float, float]]]:
    """Find contiguous arc segments of inliers along the circle (exact parallel to line_segment.find)."""
    if not inliers:
        return []
    
    # Get angle (in degrees) for each point
    def get_angle(p: tuple[float, float]) -> float:
        return math.degrees(math.atan2(p[1] - cy, p[0] - cx))
    
    angles = [get_angle(p) for p in inliers]
    
    # Sort by angle (like you sorted by projected position)
    sorted_indices = sorted(range(len(inliers)), key=lambda i: angles[i])
    sorted_points = [inliers[i] for i in sorted_indices]
    sorted_angles = [angles[i] for i in sorted_indices]
    
    arcs: list[list[tuple[float, float]]] = []
    current_arc = [sorted_points[0]]
    for i in range(1, len(sorted_points)):
        angle_diff = sorted_angles[i] - sorted_angles[i-1]
        if angle_diff <= max_angular_gap:
            current_arc.append(sorted_points[i])
        else:
            if len(current_arc) >= min_points:
                arcs.append(current_arc)
            current_arc = [sorted_points[i]]
    
    if len(current_arc) >= min_points:
        arcs.append(current_arc)
    
    return arcs

def get_angle_range(arc_points: list[tuple[float, float]], cx: float, cy: float) -> tuple[float, float]:
    """Return (start_angle, end_angle) in degrees for an arc (used when building _CircleArc)."""
    if not arc_points:
        return 0.0, 0.0
    
    def get_angle(p: tuple[float, float]) -> float:
        return math.degrees(math.atan2(p[1] - cy, p[0] - cx))
    
    angles = [get_angle(p) for p in arc_points]
    return min(angles), max(angles)

def calculate_arc_length(arc_points: list[tuple[float, float]], cx: float, cy: float, r: float) -> float:
    """Calculate arc length in the same units as the radius (parallel to calculate_length)."""
    if len(arc_points) < 2:
        return 0.0
    start_angle, end_angle = get_angle_range(arc_points, cx, cy)
    delta_angle_rad = math.radians(end_angle - start_angle)
    return r * abs(delta_angle_rad)