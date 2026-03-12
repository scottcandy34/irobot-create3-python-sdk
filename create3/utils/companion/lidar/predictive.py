
import math

from create3.models.robot import Position
from create3.utils.robot.constraints import RADIUS

def circle_to_wall_distance(wall: tuple[float, tuple[float, float], tuple[float, float]], position: Position) -> tuple[bool, float, float]:
    """
    Calculate the distance to a wall along a circle's heading, the angle between the heading and the wall,
    and whether the wall is in the circle's path.
    
    Parameters:
    - wall: tuple (wall_length, (m, b), (xmin, xmax))
            wall_length: length of the wall (not used directly)
            (m, b): slope and y-intercept of the wall's line equation y = mx + b
            (xmin, xmax): x-range where the wall exists
    - position: tuple (x, y, heading)
            (x, y): coordinates of the circle's center
            heading: direction in radians the circle is facing
    
    Returns:
    - distance: float, distance to the wall if in path, else float('inf')
    - angle: float, angle in radians between the circle's heading and the wall
    - in_path: bool, True if the wall is in the circle's path, False otherwise
    """
    # Unpack inputs
    wall_length, (m, b), (xmin, xmax) = wall
    x = position.x
    y = position.y
    heading = math.radians(position.angle)
    
    # Precompute terms
    A = m * math.cos(heading) - math.sin(heading)  # Component for angle and distance
    B = m * x - y + b                              # Signed distance term
    sqrt_m2_1 = math.sqrt(m**2 + 1)                # Normalizer
    
    # Calculate angle between heading and wall (acute angle)
    angle = math.asin(abs(A) / sqrt_m2_1)
    
    # Check if circle is already intersecting the wall
    x_proj = (x + m * y - m * b) / (1 + m**2)
    if xmin <= x_proj <= xmax:
        distance_to_line = abs(B) / sqrt_m2_1
    else:
        distance_to_line = float('inf')
    P1_x, P1_y = xmin, m * xmin + b
    P2_x, P2_y = xmax, m * xmax + b
    distance_to_P1 = math.sqrt((x - P1_x)**2 + (y - P1_y)**2)
    distance_to_P2 = math.sqrt((x - P2_x)**2 + (y - P2_y)**2)
    distance_to_segment = min(distance_to_line, distance_to_P1, distance_to_P2)
    
    if distance_to_segment <= RADIUS:
        return 0.0, angle, True
    
    # Find potential distances where circle touches the wall
    candidates = []
    
    # Case 1: Circle touches the infinite line
    if A != 0:  # Heading not parallel to wall
        for sign in [1, -1]:
            t = (sign * RADIUS * sqrt_m2_1 - B) / A
            if t > 0:
                x_c = x + t * math.cos(heading)
                y_c = y + t * math.sin(heading)
                x_proj = (x_c + m * y_c - m * b) / (1 + m**2)
                if xmin <= x_proj <= xmax:
                    candidates.append(t)
    
    # Case 2: Circle touches the endpoints
    for p_x in [xmin, xmax]:
        p_y = m * p_x + b
        dx = x - p_x
        dy = y - p_y
        coeff_b = 2 * (dx * math.cos(heading) + dy * math.sin(heading))
        coeff_c = dx**2 + dy**2 - RADIUS**2
        discriminant = coeff_b**2 - 4 * coeff_c
        if discriminant >= 0:
            sqrt_disc = math.sqrt(discriminant)
            t1 = (-coeff_b + sqrt_disc) / 2
            t2 = (-coeff_b - sqrt_disc) / 2
            if t1 > 0:
                candidates.append(t1)
            if t2 > 0:
                candidates.append(t2)
    
    # Determine distance and path
    if candidates:
        distance = min(candidates)
        in_path = True
    else:
        distance = float('inf')
        in_path = False
    
    return in_path, distance, angle
