#
# Servo Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with servo control, including setting angles and speeds."""

import math

MIN_ANGLE = 0.0
MAX_ANGLE = 180.0
SPEED_PER_DEGREE = 0.3 / 60 # seconds per degree, adjust as needed
MAX_ANGULAR_VELOCITY = math.radians(180) / (180 * SPEED_PER_DEGREE)

def validate_speed(speed: float | int) -> float:
    """Speed is magnitude only (rad/s)."""
    if speed < 0:
        speed = abs(speed)          # or raise error; direction is handled separately
    return min(speed, MAX_ANGULAR_VELOCITY)

def validate_angle(angle: float | int) -> float:
    return max(MIN_ANGLE, min(MAX_ANGLE, angle))
