#
# Joy Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with joystick inputs to control the iRobot Create3."""

import math

from geometry_msgs.msg import Twist

from create3.utils.robot import constraints as _constraints

def get_twist(x: float, y: float) -> Twist:
    """Converts joystick inputs (x and y) into a Twist message for controlling the robot's movement."""
    
    twist = Twist()
    x = x * -1  # inverses X axis direction to -->
    if y:
        xc = x * math.sqrt(1 - (1/2 * y**2))  # Map square to circle and find X
        yc = y * math.sqrt(1 - (1/2 * x**2))  # Map square to circle and find Y
        # find_radius * robot_speed_in_meters * forward/reverse_+/-
        twist.linear.x = math.sqrt(xc**2 + yc**2) * (_constraints.MAX_SPEED / 100) * (y / abs(y))  # m/s

    if x:
        k = (4 * _constraints.MAX_SPEED) / (math.pi * _constraints.WHEEL_DISTANCE_APART)  # % different
        # (find_angle - Move_by_quarter_circle) * k * left/right_+/-
        twist.angular.z = (math.atan(abs(1 / x)) - (math.pi / 2)) * k * (x / abs(x))  # rad/s

    return twist