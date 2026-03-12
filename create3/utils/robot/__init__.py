#
# Robot Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with the iRobot Create3, including converting between quaternions and euler angles, and providing access to utility classes for working with the lightring, IR sensors, and robot constraints."""

import math as _math

from . import constraints, ir, lightring

def convert_to_euler(x: int | float, y: int | float, z: int | float, w: int | float) -> tuple[float, float, float]:
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = _math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = _math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = _math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians
    
def convert_to_quaternion(roll_x: int | float, pitch_y: int | float, yaw_z: int | float) -> tuple[float, float, float, float]:
    """
    Convert a euler angle into quaternion (x, y, z, w)
    input in radians
    """

    w = round(_math.cos(roll_x / 2)* _math.cos(pitch_y / 2) * _math.cos(yaw_z / 2) + _math.sin(roll_x / 2) * _math.sin(pitch_y / 2) * _math.sin(yaw_z / 2), 15)
    x = round(_math.sin(roll_x / 2)* _math.cos(pitch_y / 2) * _math.cos(yaw_z / 2) - _math.cos(roll_x / 2) * _math.sin(pitch_y / 2) * _math.sin(yaw_z / 2), 15)
    y = round(_math.cos(roll_x / 2)* _math.sin(pitch_y / 2) * _math.cos(yaw_z / 2) + _math.sin(roll_x / 2) * _math.cos(pitch_y / 2) * _math.sin(yaw_z / 2), 15)
    z = round(_math.cos(roll_x / 2)* _math.cos(pitch_y / 2) * _math.sin(yaw_z / 2) - _math.sin(roll_x / 2) * _math.sin(pitch_y / 2) * _math.cos(yaw_z / 2), 15)
    
    return x, y, z, w