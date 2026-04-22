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
    """Convert angular speed to a positive magnitude and clamp it to the robot's maximum limit (rad/s).

    This function treats the input as *magnitude only*. Negative values are automatically
    converted to positive because direction is expected to be handled separately
    (usually by the sign of `twist.angular.z`).

    Parameters
    ----------
    speed : float | int
        Desired angular speed in rad/s (can be negative).

    Returns
    -------
    float
        Clamped speed in the range [0.0, MAX_ANGULAR_VELOCITY].
    """
    # Convert to positive magnitude (direction handled elsewhere)
    speed = abs(float(speed))
    return min(speed, MAX_ANGULAR_VELOCITY)

def validate_angle(angle: float | int) -> float:
    """Clamp an angle to the valid range [MIN_ANGLE, MAX_ANGLE].

    Useful for limiting turn angles in navigation, waypoint following,
    or any motion control logic.

    Parameters
    ----------
    angle : float | int
        Angle value to validate (in whatever units your code uses).

    Returns
    -------
    float
        Angle clamped between MIN_ANGLE and MAX_ANGLE.
    """
    return max(MIN_ANGLE, min(MAX_ANGLE, float(angle)))