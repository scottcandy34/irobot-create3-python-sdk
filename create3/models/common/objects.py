#
# Common Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

@dataclass
class QuaternionAngles:
    """Stores Quaternion coordinate vectors"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

@dataclass
class EulerAngles:
    """Stores Euler angles coordinate vectors"""
    roll_x: float = 0.0
    pitch_y: float = 0.0
    yaw_z: float = 0.0

@dataclass
class Position:
    """Stores position values."""
    x: int | float = 0.0
    y: int | float = 0.0
    angle: int | float = 0.0

@dataclass
class Direction:
    distance: float = 0.0
    angle: float = 0.0

@dataclass
class RansacConfig:
    """Unified configuration for RANSAC and MSAC line/circle detection."""
    max_iterations: int = 0
    distance_threshold: float = 0.0      # max distance for inlier / MSAC cutoff
    min_inliers: int = 0
    max_gap: float = 0.0                 # lines: distance gap | circles: angular gap (degrees)
    min_points: int = 0                 # min points per segment or arc