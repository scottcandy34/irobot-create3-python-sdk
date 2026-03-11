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

from . import line, line_segment
from ...robot import constraints as _constraints
from ...robot import lightring as _lightring
from ....models import Position as _Position

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

def get_coords(lidar_scans: list[float], index: int, angle_increment: float, robot_position: _Position) -> tuple[float, float]:
    """Convert a lidar scan at a specific index to (x, y) coordinates in the robot's frame of reference."""

    distance = lidar_scans[index]
    if _math.isinf(distance):
        return None
    angle = angle_increment * index
    x = -distance * _math.cos(angle) + robot_position.x
    y = distance * _math.sin(angle) + robot_position.y
    return (x, y)

def find_lines_and_segments(points: list[tuple[float, float]], max_iterations = 100, distance_threshold = 1, min_inliers = 30, max_gap = 5, min_points_per_segment = 30) -> list[tuple[float, tuple[float, float], tuple[float, float]]]:
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
    results = []
    
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
                results.append((length, (m, b), (xmin, xmax)))
        
        remaining_points = [point for point in remaining_points if point not in best_inliers]
    
    return results
