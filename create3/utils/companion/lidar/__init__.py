#
# Lidar Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""
Tools for working with the lidar sensor on the iRobot Create3, including converting scans to coordinates, 
finding lines and segments in the data, and generating lightring patterns based on lidar readings.
"""

import math as _math
import random as _random

from irobot_create_msgs.msg import LedColor as _LedColor

from . import line, line_segment, predictive, circle, circle_arc
from create3.utils.robot import constraints as _constraints, lightring as _lightring
from create3.models.robot import Position as _Position
from create3.models.companion import Wall as _Wall, Lidar as _Lidar, Column as _Column

def get_motion_lightring(lidar_scans: list[float], red: int = None, green: int = None, blue: int = None) -> list[_LedColor]:
    """Returns a list of LEDs that are highlighted based on the closest lidar scan."""
    
    if lidar_scans and min(lidar_scans) < 35:
        scans = lidar_scans
        rotation = scans.index(min(scans)) / len(scans) # Returns percentage between 0.0 to 1.
        
        if red is not None and green is not None and blue is not None:
            led = _LedColor(red=red, green=green, blue=blue)
        else:
            led = _lightring.get_hue_percentage(rotation)
        
        lightring_leds = []
        for i in range(6):
            lightring_leds += [_lightring.adjust_rotation_brightness(led, rotation, _constraints.get_led_angle(i))]
            
        return lightring_leds
        
    return None

def get_coords(lidar: _Lidar, index: int, robot_position: _Position) -> tuple[float, float]:
    """Convert a lidar scan at a specific index to (x, y) coordinates in the robot's frame of reference."""

    distance = lidar.ranges[index]
    if _math.isinf(distance):
        return None
    
    # 1. Angle of this ray in robot frame (radians)
    phi_deg = lidar.angle_min + index * lidar.angle_increment
    phi = _math.radians(phi_deg)

    # 2. Local point in robot frame (X forward, Y left)
    local_x = distance * _math.cos(phi)
    local_y = distance * _math.sin(phi)

    # 3. Robot heading in radians
    theta = _math.radians(robot_position.angle)

    # 4. Rotate to world frame + translate
    world_x = robot_position.x + local_x * _math.cos(theta) - local_y * _math.sin(theta)
    world_y = robot_position.y + local_x * _math.sin(theta) + local_y * _math.cos(theta)

    return (world_x, world_y)

def find_lines_and_segments(points: list[tuple[float, float]], max_iterations = 100, distance_threshold = 1, min_inliers = 30, max_gap = 5, min_points_per_segment = 30) -> list[_Wall]:
    """
    Find lines and their segments in a set of 2D points using RANSAC, returning x-limits for each segment.
    
    Args:
        points: List of (x, y) tuples.
        max_iterations: Maximum RANSAC iterations per line.
        distance_threshold: Max distance from line for a point to be an inlier.
        min_inliers: Minimum number of inliers to accept a line.
        max_gap: Maximum gap between points along the line to be in the same segment.
        min_points_per_segment: Minimum number of points required to form a segment.
    
    Returns:
        List of tuples: (segment length, (slope m, intercept b), (xmin, xmax))
    """

    remaining_points = points.copy()
    results: list[_Wall] = []

    while len(remaining_points) >= min_inliers:
        best_inliers = []
        for _ in range(max_iterations):
            p1, p2 = _random.sample(remaining_points, 2)
            try:
                m, b = line.fit_line([p1, p2])
                inliers = [point for point in remaining_points if line.distance_to_line(point, m, b) < distance_threshold]
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
            except ValueError:
                continue

        if len(best_inliers) < min_inliers:
            break

        m, b = line.fit_line(best_inliers)
        segments = line_segment.find(best_inliers, m, b, max_gap, min_points=min_points_per_segment)
        
        for segment in segments:
            if len(segment) >= min_points_per_segment:
                # Project the first and last points onto the line
                proj_first = line_segment.project_point(segment[0], m, b)
                proj_last = line_segment.project_point(segment[-1], m, b)
                xmin = proj_first[0]
                xmax = proj_last[0]
                length = line_segment.calculate_length(segment, m, b)
                
                wall = _Wall(length=length, slope=m, intercept=b, xmin=min(xmin, xmax), xmax=max(xmin, xmax))
                results.append(wall)
        
        remaining_points = [point for point in remaining_points if point not in best_inliers]
    
    return results

def find_circles_and_arcs(points: list[tuple[float, float]], max_iterations=100, distance_threshold=1, min_inliers=30, max_angular_gap=15, min_points_per_arc=30) -> list[_Column]:
    """
    Find circles and their arcs in a set of 2D points using RANSAC.
    Mirrors your find_lines_and_segments exactly in structure, loop logic,
    inlier removal, and parameter names.
    
    Returns:
        List of _CircleArc objects (one per detected arc).
    """

    remaining_points = points.copy()
    results: list[_Column] = []

    while len(remaining_points) >= min_inliers:
        best_inliers = []
        best_params = None

        for _ in range(max_iterations):
            if len(remaining_points) < 3:
                break
            p1, p2, p3 = _random.sample(remaining_points, 3)
            try:
                cx, cy, r = circle.fit_circle([p1, p2, p3])          # sample 3 points
                inliers = [point for point in remaining_points 
                           if circle.distance_to_circle(point, cx, cy, r) < distance_threshold]
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_params = (cx, cy, r)
            except ValueError:
                continue

        if len(best_inliers) < min_inliers or best_params is None:
            break

        # Refit using ALL inliers (exactly like you do with fit_line on best_inliers)
        cx, cy, r = circle.fit_circle(best_inliers)

        # Find contiguous arcs (parallel to line_segment.find)
        arcs = circle_arc.find(best_inliers, cx, cy, r, max_angular_gap, min_points=min_points_per_arc)
        
        for arc_points in arcs:
            if len(arc_points) >= min_points_per_arc:
                start_angle, end_angle = circle_arc.get_angle_range(arc_points, cx, cy)
                arc_length = circle_arc.calculate_arc_length(arc_points, cx, cy, r)

                arc = _Column(cx=cx, cy=cy, radius=r, start_angle=start_angle, end_angle=end_angle, arc_length=arc_length)
                results.append(arc)
        
        # Remove inliers from remaining points (exact same logic as your line version)
        remaining_points = [point for point in remaining_points if point not in best_inliers]
    
    return results