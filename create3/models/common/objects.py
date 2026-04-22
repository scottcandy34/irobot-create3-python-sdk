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