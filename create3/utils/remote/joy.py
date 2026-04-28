#
# Joy Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with joystick inputs to control the iRobot Create3."""

import math

from geometry_msgs.msg import Twist

from create3.utils.robot import constraints as _constraints

def get_twist(x: float, y: float) -> Twist:
    """Convert normalized joystick inputs (x, y) into a ROS Twist for differential-drive control.

    This mapping is specifically tuned for arcade-style driving:
      • Square-to-circle transformation prevents speed loss at the diagonals
      • X axis is inverted to match typical robot coordinate conventions
      • Linear speed is scaled to _constraints.MAX_SPEED (in m/s)
      • Angular speed is scaled so that full stick deflection produces
        maximum wheel-speed difference

    Parameters
    ----------
    x : float
        Normalized joystick X axis (-1.0 = left, +1.0 = right).
    y : float
        Normalized joystick Y axis (-1.0 = reverse, +1.0 = forward).

    Returns
    -------
    Twist
        cmd_vel message with linear.x (m/s) and angular.z (rad/s).
    """
    # Clamp inputs to the valid joystick range
    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))

    twist = Twist()

    # Invert X axis (common for robot coordinate systems)
    x = -x

    # --- Linear velocity (forward/backward) ---
    if abs(y) > 1e-6:
        # Square-to-circle mapping (keeps max speed the same in all directions)
        xc = x * math.sqrt(1.0 - 0.5 * y**2)
        yc = y * math.sqrt(1.0 - 0.5 * x**2)

        # Magnitude gives speed, sign of y gives direction
        magnitude = math.sqrt(xc**2 + yc**2)
        twist.linear.x = magnitude * (_constraints.MAX_SPEED / 100.0) * (y / abs(y))

    # --- Angular velocity (turning) ---
    if abs(x) > 1e-6:
        k = (4.0 * _constraints.MAX_SPEED) / (math.pi * _constraints.WHEEL_DISTANCE_APART)

        # Custom mapping: atan(1/|x|) creates a smooth turn curve
        turn = math.atan(abs(1.0 / x)) - (math.pi / 2.0)
        twist.angular.z = turn * k * (x / abs(x))

    return twist