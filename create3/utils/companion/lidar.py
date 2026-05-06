#
# LiDAR Tools for iRobot Create3 - Jazzy
# =====================================================================
# Created by scottcandy34 • NumPy Optimized & Revised April 2026
#
# High-performance utilities for processing LiDAR data on the companion node.
#
# This module provides two core capabilities used throughout the perception
# pipeline:
#   • Motion-compensated (deskewed) point cloud generation
#   • Reactive lightring visualization based on closest obstacle direction
#
# All heavy lifting is fully vectorized with NumPy for real-time performance
# even on a Raspberry Pi.
# =====================================================================

"""
LiDAR utilities for the iRobot Create3 companion node.

Provides essential tools for:
  • Converting raw LiDAR scans into accurate world-frame point clouds
    with motion compensation (deskewing) using pose history.
  • Generating dynamic lightring patterns that visually indicate the
    direction of the closest obstacle.

These functions are heavily used by the `GENERATE_COORDS` task and
the `LIDAR_LIGHTRING` task.
"""

import numpy as np
from irobot_create_msgs.msg import LedColor

from create3.models.common import Position, Stamped
from create3.models.companion import Lidar
from create3.utils.robot import constraints, lightring


def get_motion_lightring(
    lidar_scans: list[float],
    red: int | None = None,
    green: int | None = None,
    blue: int | None = None,
) -> list[LedColor] | None:
    """Return a 6-LED lightring pattern highlighting the direction of the closest obstacle.

    Lights up the LED(s) nearest the minimum distance in the LiDAR scan
    when that distance is < 35 cm. Uses the same spotlight-style falloff
    as the IR-based lightring task.

    Parameters
    ----------
    lidar_scans : list[float]
        List of LiDAR distance measurements in centimeters.
    red, green, blue : int | None, optional
        Fixed RGB color (0–255). If any value is None, a hue-based color
        is automatically chosen based on obstacle angle.

    Returns
    -------
    list[LedColor] | None
        List of 6 `LedColor` objects for the lightring, or None if no
        obstacle is closer than 35 cm (or the scan is empty).
    """
    if not lidar_scans or min(lidar_scans) >= 35:
        return None

    # Find normalized direction of closest point (0.0–1.0)
    min_dist = min(lidar_scans)
    rotation = lidar_scans.index(min_dist) / len(lidar_scans)

    # Choose base LED color
    if red is not None and green is not None and blue is not None:
        base_led = LedColor(red=red, green=green, blue=blue)
    else:
        base_led = lightring.get_hue_percentage(rotation)

    # Build final lightring pattern with angular brightness falloff
    lightring_leds: list[LedColor] = []
    for i in range(6):
        led_color = lightring.adjust_rotation_brightness(
            base_led, rotation, constraints.get_led_angle(i)
        )
        lightring_leds.append(led_color)

    return lightring_leds


def deskew_lidar_scan(
    lidar_stamped: Stamped[Lidar], pose_history: list[Stamped[Position]]
) -> list[tuple[float, float]]:
    """Convert a raw LiDAR scan into a motion-compensated world-frame point cloud.

    This is the core of the `GENERATE_COORDS` task. It interpolates the robot's
    pose at the exact timestamp of each LiDAR ray and transforms all points
    from the robot's local frame into world coordinates.

    Fully vectorized with NumPy for excellent performance.

    Parameters
    ----------
    lidar_stamped : Stamped[Lidar]
        Timestamped LiDAR scan data.
    pose_history : list[Stamped[Position]]
        Recent pose history from the HISTORY_KEEPER task (used for interpolation).

    Returns
    -------
    list[tuple[float, float]]
        List of (x, y) points in world coordinates (cm).
        Returns empty list if the scan contains no valid readings.
    """
    lidar = lidar_stamped.data

    if not lidar.ranges or len(lidar.ranges) == 0:
        return []

    # === 1. Filter valid ranges ===
    ranges = np.asarray(lidar.ranges, dtype=np.float64)
    valid_mask = (
        (ranges >= lidar.range_min)
        & (ranges <= lidar.range_max)
        & np.isfinite(ranges)
    )

    if not np.any(valid_mask):
        return []

    valid_idx = np.flatnonzero(valid_mask)
    valid_ranges = ranges[valid_mask]

    # === 2. Compute local (x, y) coordinates for valid rays ===
    angles_deg = lidar.angle_min + valid_idx * lidar.angle_increment
    angles_rad = np.deg2rad(angles_deg)

    local_x = valid_ranges * np.cos(angles_rad)
    local_y = valid_ranges * np.sin(angles_rad)

    # === 3. Compute precise timestamps for each ray ===
    start_ns = lidar_stamped.timestamp.nanoseconds
    dt_per_ray_ns = int(round(lidar.time_increment * 1_000_000_000))
    ray_ns = start_ns + valid_idx.astype(np.int64) * dt_per_ray_ns

    # === 4. No pose history → simple local-to-world fallback ===
    if not pose_history:
        return list(zip(local_x.tolist(), local_y.tolist()))

    # === 5. Prepare pose history as NumPy arrays ===
    pose_ns = np.array([p.timestamp.nanoseconds for p in pose_history], dtype=np.int64)
    pose_x = np.array([p.data.x for p in pose_history], dtype=np.float64)
    pose_y = np.array([p.data.y for p in pose_history], dtype=np.float64)
    pose_angle = np.array([p.data.angle for p in pose_history], dtype=np.float64)

    # === 6. Vectorized pose interpolation using searchsorted ===
    idx = np.searchsorted(pose_ns, ray_ns, side="right") - 1
    idx = np.clip(idx, 0, len(pose_history) - 1)

    before_x = pose_x[idx]
    before_y = pose_y[idx]
    before_a = pose_angle[idx]
    before_t = pose_ns[idx]

    after_idx = np.minimum(idx + 1, len(pose_history) - 1)
    after_x = pose_x[after_idx]
    after_y = pose_y[after_idx]
    after_a = pose_angle[after_idx]
    after_t = pose_ns[after_idx]

    # Interpolation factor t ∈ [0, 1]
    dt_total = (after_t - before_t).astype(np.float64)
    dt_target = (ray_ns - before_t).astype(np.float64)
    t = np.zeros_like(dt_total)
    mask = dt_total > 0
    t[mask] = dt_target[mask] / dt_total[mask]

    # Interpolate pose
    interp_x = before_x + t * (after_x - before_x)
    interp_y = before_y + t * (after_y - before_y)
    interp_angle = before_a + t * (after_a - before_a)

    # Normalize angle to [-180, 180]
    interp_angle = (interp_angle + 180) % 360 - 180

    # === 7. Vectorized local → world transform ===
    theta_rad = np.deg2rad(interp_angle)
    cos_theta = np.cos(theta_rad)
    sin_theta = np.sin(theta_rad)

    world_x = interp_x + local_x * cos_theta - local_y * sin_theta
    world_y = interp_y + local_x * sin_theta + local_y * cos_theta

    return list(zip(world_x.tolist(), world_y.tolist()))