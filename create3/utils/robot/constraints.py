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

def get_ir_angle(index: int) -> float:
    """Return the angle for IR sensor location"""

    angle: float = None
    match index:
        case 0:
            angle = 130.6
        case 1:
            angle = 103.3
        case 2:
            angle = 85.3
        case 3:
            angle = 68.3
        case 4:
            angle = 51.05
        case 5:
            angle = 31.3
        case 6:
            angle = 0.0
            
    return angle
            
def get_led_angle(self, index: int) -> float:
    """Return the angle for LED location"""
    
    angle = 0.0
    match index:
        case 0:
            angle = 60.0
        case 1:
            angle = 120.0
        case 2:
            angle = 180.0
        case 3:
            angle = 240.0
        case 4:
            angle = 300.0
        case 5:
            angle = 0.0
            
    return angle