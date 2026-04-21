#
# Predictive Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math

from create3.models.common import Position
from create3.utils.robot.constraints import RADIUS
from create3.models.companion import Wall, Interaction

def circle_to_wall_distance(wall: Wall, position: Position) -> Interaction:
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
    x = position.x
    y = position.y
    heading = math.radians(position.angle)
    
    # Precompute terms
    A = wall.slope * math.cos(heading) - math.sin(heading)  # Component for angle and distance
    B = wall.slope * x - y + wall.intercept                              # Signed distance term
    sqrt_m2_1 = math.sqrt(wall.slope**2 + 1)                # Normalizer
    
    # Calculate angle between heading and wall (acute angle)
    angle = math.asin(abs(A) / sqrt_m2_1)
    
    # Check if circle is already intersecting the wall
    x_proj = (x + wall.slope * y - wall.slope * wall.intercept) / (1 + wall.slope**2)
    if wall.xmin <= x_proj <= wall.xmax:
        distance_to_line = abs(B) / sqrt_m2_1
    else:
        distance_to_line = float('inf')
    P1_x, P1_y = wall.xmin, wall.slope * wall.xmin + wall.intercept
    P2_x, P2_y = wall.xmax, wall.slope * wall.xmax + wall.intercept
    distance_to_P1 = math.sqrt((x - P1_x)**2 + (y - P1_y)**2)
    distance_to_P2 = math.sqrt((x - P2_x)**2 + (y - P2_y)**2)
    distance_to_segment = min(distance_to_line, distance_to_P1, distance_to_P2)
    
    if distance_to_segment <= RADIUS:
        return Interaction(angle=angle, distance=0.0, in_path=True)
    
    # Find potential distances where circle touches the wall
    candidates = []
    
    # Case 1: Circle touches the infinite line
    if A != 0:  # Heading not parallel to wall
        for sign in [1, -1]:
            t = (sign * RADIUS * sqrt_m2_1 - B) / A
            if t > 0:
                x_c = x + t * math.cos(heading)
                y_c = y + t * math.sin(heading)
                x_proj = (x_c + wall.slope * y_c - wall.slope * wall.intercept) / (1 + wall.slope**2)
                if wall.xmin <= x_proj <= wall.xmax:
                    candidates.append(t)
    
    # Case 2: Circle touches the endpoints
    for p_x in [wall.xmin, wall.xmax]:
        p_y = wall.slope * p_x + wall.intercept
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

    return Interaction(angle=angle, distance=distance, in_path=in_path)
