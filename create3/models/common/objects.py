#
# Common Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

@dataclass
class QuaternionAngles:
    """Stores a quaternion orientation (x, y, z, w)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

@dataclass
class EulerAngles:
    """Stores Euler angles (roll, pitch, yaw) in radians."""

    roll_x: float = 0.0
    pitch_y: float = 0.0
    yaw_z: float = 0.0

@dataclass
class Position:
    """Stores a 2D robot position and heading.

    Units:
        x, y    → centimeters
        angle   → degrees
    """

    x: int | float = 0.0
    y: int | float = 0.0
    angle: int | float = 0.0

@dataclass
class Direction:
    """Stores the direction and distance to a target point."""

    distance: float = 0.0
    angle: float = 0.0

@dataclass
class RansacConfig:
    """Unified configuration for RANSAC and MSAC line/circle detection."""

    max_iterations: int = 0
    distance_threshold: float = 0.0      # maximum distance for a point to be considered an inlier
    min_inliers: int = 0                 # minimum number of inliers required to accept a model
    max_gap: float = 0.0                 # lines: max distance gap | circles: max angular gap (degrees)
    min_points: int = 0                  # minimum points required for a valid segment/arc