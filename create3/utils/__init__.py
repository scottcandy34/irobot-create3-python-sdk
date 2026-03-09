#
# Utility Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""
Tools for working with the iRobot Create3, including converting between quaternions and euler angles, 
processing joystick input data, and providing access to utility classes for working with the lightring, IR sensors, 
line fitting, line segments, joystick input, and lidar data.
"""

from . import robot, rpi, pc
# import rclpy_override as rclpy
from .rclpy_override import rclpy

import pprint as _pprint

def object_to_string(obj) -> str:
    """Returns a pretty string with the object data"""

    if isinstance(obj, str):
        return obj

    if hasattr(obj, '__dict__'):
        data = vars(obj)
    else:
        data = obj

    return _pprint.pformat(data, indent = 4, width = 80)