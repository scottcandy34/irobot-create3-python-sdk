#
# Predictive Collision Tools for iRobot Create3 - Jazzy
# =====================================================================
# Created by scottcandy34 • Revised & Fully Documented April 2026
#
# High-performance geometric collision prediction between the circular
# robot body and detected wall segments.
#
# This module is used by navigation and obstacle avoidance tasks to
# predict the first point of contact along the robot's current heading.
#
# Key Features:
#   • Accurate circle-to-finite-line-segment intersection
#   • Handles already-intersecting cases, infinite line hits, and endpoint hits
#   • Returns Interaction object with angle, distance, and in-path flag
#
# Performance Note: Pure scalar math — called per detected wall, so remains very fast.
# =====================================================================

import math

from create3.models.common import Position
from create3.utils.robot.constraints import RADIUS
from create3.models.companion import Wall, Interaction


def circle_to_wall_distance(wall: Wall, position: Position) -> Interaction:
    """Calculate the first collision distance (if any) between the circular robot
    and a finite wall segment.

    This is the core predictive collision function used for safe navigation
    and wall-following behaviors. It determines:

      • The acute angle between the robot's heading and the wall
      • The distance along the heading until the circle first touches the wall
      • Whether a collision occurs at all

    It intelligently handles three geometric cases:
      1. Robot is already intersecting the wall (distance = 0)
      2. Circle touches the infinite line of the wall
      3. Circle touches one of the wall's endpoints

    Parameters
    ----------
    wall : Wall
        Detected wall segment containing slope, intercept, and x-range.
    position : Position
        Current robot pose (x, y in cm, angle in degrees).

    Returns
    -------
    Interaction
        Container with:
        - angle (rad): acute angle between robot heading and wall
        - distance (float): collision distance along heading (or inf if none)
        - in_path (bool): True if the wall lies in the robot's future path
    """
    # Unpack robot pose
    x = position.x
    y = position.y
    heading_rad = math.radians(position.angle)

    # Precompute line coefficients
    A = wall.slope * math.cos(heading_rad) - math.sin(heading_rad)
    B = wall.slope * x - y + wall.intercept
    normalizer = math.sqrt(wall.slope**2 + 1.0)

    # Acute angle between heading and wall
    angle = math.asin(abs(A) / normalizer)

    # ------------------------------------------------------------------
    # 1. Check if robot is already intersecting the wall segment
    # ------------------------------------------------------------------
    denom = 1.0 + wall.slope**2
    x_proj = (x + wall.slope * y - wall.slope * wall.intercept) / denom

    if wall.xmin <= x_proj <= wall.xmax:
        distance_to_line = abs(B) / normalizer
    else:
        distance_to_line = float("inf")

    # Distance to the two wall endpoints
    P1_x = wall.xmin
    P1_y = wall.slope * P1_x + wall.intercept
    P2_x = wall.xmax
    P2_y = wall.slope * P2_x + wall.intercept

    dist_to_P1 = math.hypot(x - P1_x, y - P1_y)
    dist_to_P2 = math.hypot(x - P2_x, y - P2_y)

    distance_to_segment = min(distance_to_line, dist_to_P1, dist_to_P2)

    if distance_to_segment <= RADIUS:
        return Interaction(angle=angle, distance=0.0, in_path=True)

    # ------------------------------------------------------------------
    # 2. Find future intersection distances (t > 0) along the heading ray
    # ------------------------------------------------------------------
    candidates: list[float] = []

    # Case 1: Intersection with the infinite line
    if abs(A) > 1e-10:  # not parallel
        for sign in (1.0, -1.0):
            t = (sign * RADIUS * normalizer - B) / A
            if t > 0:
                # Check if contact point lies within wall segment
                x_c = x + t * math.cos(heading_rad)
                y_c = y + t * math.sin(heading_rad)
                x_proj = (x_c + wall.slope * y_c - wall.slope * wall.intercept) / denom
                if wall.xmin <= x_proj <= wall.xmax:
                    candidates.append(t)

    # Case 2: Intersection with the two finite endpoints
    for p_x in (wall.xmin, wall.xmax):
        p_y = wall.slope * p_x + wall.intercept
        dx = x - p_x
        dy = y - p_y

        b_coeff = 2.0 * (dx * math.cos(heading_rad) + dy * math.sin(heading_rad))
        c_coeff = dx**2 + dy**2 - RADIUS**2
        discriminant = b_coeff**2 - 4.0 * c_coeff

        if discriminant >= 0:
            sqrt_disc = math.sqrt(discriminant)
            t1 = (-b_coeff + sqrt_disc) / 2.0
            t2 = (-b_coeff - sqrt_disc) / 2.0
            if t1 > 0:
                candidates.append(t1)
            if t2 > 0:
                candidates.append(t2)

    # ------------------------------------------------------------------
    # Final result
    # ------------------------------------------------------------------
    if candidates:
        distance = min(candidates)
        in_path = True
    else:
        distance = float("inf")
        in_path = False

    return Interaction(angle=angle, distance=distance, in_path=in_path)
