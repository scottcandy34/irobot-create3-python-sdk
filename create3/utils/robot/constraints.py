#
# Constraints for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Constants for working with the iRobot Create3, including physical dimensions of the robot and functions for calculating angles for the IR sensors and lightring LEDs based on their index."""

WHEEL_DISTANCE_APART = 23.5 # cm
"""This is the distance between the two wheels. 23.5cm"""
WHEEL_RADIUS = 3.6 # cm
"""This is the radius of the wheels. 3.6cm"""
RADIUS = 16.2
"""This is the radius of the robot from the center to the edge. 16.2cm"""
MAX_SPEED = 46 # cm/s
"""This is the maximum speed of the robot. 46cm/s"""
MAX_ANGULAR_SPEED = MAX_SPEED / RADIUS # rad/s
"""This is the maximum angular speed of the robot, calculated based on the max linear speed and the radius. ~2.84 rad/s"""

def get_ir_angle(index: int) -> float | None:
    """Return the angle (in degrees) for the given IR sensor index (0-6).

    These angles represent the physical direction of each of the 7 IR sensors
    relative to the robot's forward axis (index 6 = 0°).

    Index layout:
      0 → 130.6°    1 → 103.3°    2 → 85.3°
      3 → 68.3°     4 → 51.05°    5 → 31.3°
      6 → 0.0°

    Returns None for any index outside 0–6.
    """
    ir_angles = {
        0: 130.6,
        1: 103.3,
        2: 85.3,
        3: 68.3,
        4: 51.05,
        5: 31.3,
        6: 0.0,
    }
    return ir_angles.get(index)

def get_led_angle(index: int) -> float:
    """Return the angle (in degrees) for the given LED index (0-5) on the 6-LED lightring.

    LEDs are placed evenly at 60° intervals around the robot.

    Index layout:
      0 → 60.0°     1 → 120.0°    2 → 180.0°
      3 → 240.0°    4 → 300.0°    5 → 0.0°

    Returns 0.0° for any index outside 0–5 (safe default).
    """
    led_angles = {
        0: 60.0,
        1: 120.0,
        2: 180.0,
        3: 240.0,
        4: 300.0,
        5: 0.0,
    }
    return led_angles.get(index, 0.0)