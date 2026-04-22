#
# Lidar Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""
Tools for working with the lidar sensor on the iRobot Create3, including converting scans to coordinates
and generating lightring patterns based on lidar readings.
"""

import math

from irobot_create_msgs.msg import LedColor

from create3.models.common import Position
from create3.utils.robot import constraints, lightring
from create3.models.companion import Lidar

def get_motion_lightring(lidar_scans: list[float], red: int | None = None, green: int | None = None, blue: int | None = None) -> list[LedColor] | None:
    """Return a 6-LED lightring pattern that highlights the direction of the closest obstacle.

    Lights up the LED(s) nearest the minimum distance in the LiDAR scan when that
    distance is < 35 cm. Uses the same spotlight falloff as the IR version.

    Parameters
    ----------
    lidar_scans : list[float]
        List of LiDAR distance measurements (in cm).
    red, green, blue : int | None
        Optional fixed RGB color (0–255). If any is None, a hue-based color
        is used based on the obstacle angle.

    Returns
    -------
    list[LedColor] | None
        List of 6 LED colors for the lightring, or None if no obstacle is closer
        than 35 cm (or scan is empty).
    """
    if not lidar_scans or min(lidar_scans) >= 35:
        return None

    # Find direction of the closest point (normalized 0.0–1.0)
    min_dist = min(lidar_scans)
    rotation = lidar_scans.index(min_dist) / len(lidar_scans)

    # Choose base LED color
    if red is not None and green is not None and blue is not None:
        led = LedColor(red=red, green=green, blue=blue)
    else:
        led = lightring.get_hue_percentage(rotation)

    # Build lightring pattern
    lightring_leds: list[LedColor] = []
    for i in range(6):
        led_color = lightring.adjust_rotation_brightness(led, rotation, constraints.get_led_angle(i))
        lightring_leds.append(led_color)

    return lightring_leds

def get_coords(lidar: Lidar, index: int, robot_position: Position) -> tuple[float, float] | None:
    """Convert a single LiDAR scan ray into world coordinates.

    Parameters
    ----------
    lidar : Lidar
        LiDAR object with .ranges, .angle_min, .angle_increment.
    index : int
        Index of the ray in the scan.
    robot_position : Position
        Robot pose with .x, .y, .angle (in degrees).

    Returns
    -------
    tuple[float, float] | None
        (world_x, world_y) in world frame, or None if the range is infinite.
    """
    distance = lidar.ranges[index]
    if math.isinf(distance):
        return None

    # Ray angle in robot frame (radians)
    phi_deg = lidar.angle_min + index * lidar.angle_increment
    phi = math.radians(phi_deg)

    # Local (robot) coordinates
    local_x = distance * math.cos(phi)
    local_y = distance * math.sin(phi)

    # Robot heading in radians
    theta = math.radians(robot_position.angle)

    # Transform to world frame
    world_x = (
        robot_position.x
        + local_x * math.cos(theta)
        - local_y * math.sin(theta)
    )
    world_y = (
        robot_position.y
        + local_x * math.sin(theta)
        + local_y * math.cos(theta)
    )

    return world_x, world_y