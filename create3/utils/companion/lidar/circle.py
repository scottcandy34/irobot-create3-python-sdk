import math

def fit_circle(points: list[tuple[float, float]]) -> tuple[float, float, float]:
    """Fit a circle to a set of points and return (cx, cy, r).
    Uses centroid + mean-radius method (simple, robust, and mirrors
    the closed-form simplicity of your fit_line)."""
    if len(points) < 3:
        raise ValueError("Need at least 3 points to fit a circle")
    
    n = len(points)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n
    r = sum(math.hypot(p[0] - cx, p[1] - cy) for p in points) / n
    return cx, cy, r


def distance_to_circle(point: tuple[float, float], cx: float, cy: float, r: float) -> float:
    """Calculate the radial distance error from a point to the circle (parallel to distance_to_line)."""
    x, y = point
    dist_from_center = math.hypot(x - cx, y - cy)
    return abs(dist_from_center - r)
