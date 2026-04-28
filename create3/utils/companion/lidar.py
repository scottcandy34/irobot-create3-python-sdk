#
# Lidar Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""
Tools for working with the lidar sensor on the iRobot Create3, including converting scans to coordinates
and generating lightring patterns based on lidar readings.
"""

import math
from typing import Optional

from rclpy.time import Time
from rclpy.duration import Duration
from irobot_create_msgs.msg import LedColor

from create3.models.common import Position, Stamped
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

def transform_local_to_world(local_x: float, local_y: float, pose: Position) -> tuple[float, float]:
    """Transform a point from robot-local frame to world frame.

    Uses the robot's current position and heading.
    """
    angle_rad = math.radians(pose.angle)

    world_x = pose.x + local_x * math.cos(angle_rad) - local_y * math.sin(angle_rad)
    world_y = pose.y + local_x * math.sin(angle_rad) + local_y * math.cos(angle_rad)

    return world_x, world_y

def interpolate_pose_at_time(pose_history: list[Stamped[Position]], target_time: Time) -> Optional[Position]:
    """Linearly interpolate the robot pose at a specific timestamp.

    Uses the pose history maintained by the `HISTORY_KEEPER` task.

    Returns None if no history is available.
    """
    if not pose_history:
        return None

    before: Stamped[Position] | None = None
    after: Stamped[Position] | None = None

    for stamped in pose_history:
        if stamped.timestamp <= target_time:
            before = stamped
        else:
            after = stamped
            break

    if before is None:
        return pose_history[0].data
    if after is None:
        return before.data

    # Time interpolation factor
    dt_total = (after.timestamp - before.timestamp).nanoseconds
    if dt_total == 0:
        return before.data

    dt_target = (target_time - before.timestamp).nanoseconds
    t = dt_target / dt_total

    p1 = before.data
    p2 = after.data

    interp = Position()
    interp.x = p1.x + t * (p2.x - p1.x)
    interp.y = p1.y + t * (p2.y - p1.y)
    interp.angle = p1.angle + t * (p2.angle - p1.angle)

    # Normalize angle to [-180, 180]
    while interp.angle > 180:
        interp.angle -= 360
    while interp.angle < -180:
        interp.angle += 360

    return interp

def deskew_lidar_scan(lidar_stamped: Stamped[Lidar], pose_history: list[Stamped[Position]]) -> list[tuple[float, float]]:
    """Motion-compensated (deskewed) conversion of a LiDAR scan into world-frame points.

    Corrects for robot movement during the scan by interpolating the robot pose
    at the exact time each individual ray was measured.

    This significantly improves accuracy for fast-moving robots or high-speed scans.
    """
    lidar = lidar_stamped.data
    scan_start_time: Time = lidar_stamped.timestamp

    if not lidar.ranges or lidar.size() == 0:
        return []

    deskewed: list[tuple[float, float]] = []
    dt_per_ray = lidar.time_increment

    for i, range_cm in enumerate(lidar.ranges):
        if range_cm is None or not (lidar.range_min <= range_cm <= lidar.range_max):
            continue

        # Timestamp of this specific ray
        ray_time = scan_start_time + Duration(seconds=i * dt_per_ray)

        # Interpolate robot pose at the exact moment this ray was measured
        current_pose = interpolate_pose_at_time(pose_history, ray_time)
        if current_pose is None:
            current_pose = Position()  # fallback to latest known pose

        # Compute local point in robot frame at measurement time
        angle = lidar.angle_min + i * lidar.angle_increment
        angle_rad = math.radians(angle)

        local_x = range_cm * math.cos(angle_rad)
        local_y = range_cm * math.sin(angle_rad)

        # Transform to world frame using interpolated pose
        world_x, world_y = transform_local_to_world(local_x, local_y, current_pose)

        deskewed.append((world_x, world_y))

    return deskewed