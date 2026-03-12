#
# IR Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with the IR sensors on the iRobot Create3."""

from irobot_create_msgs.msg import LedColor as _LedColor

from . import lightring, constraints

def get_rotation_position(ir_sensors: list[int]) -> float:
    """Returns the rotation of the IR signal as a percentage between 0.0 and 1.0 based on the 7 IR sensors."""

    maxIndex = ir_sensors.index(max(ir_sensors))

    left_angle = constraints.get_ir_angle(maxIndex - 1)
    middle_angle = constraints.get_ir_angle(maxIndex)
    right_angle = constraints.get_ir_angle(maxIndex + 1)
    
    angle: int = 0.0
    
    if ir_sensors[maxIndex] > 0:
        if left_angle and right_angle:
            if ir_sensors[maxIndex - 1] > ir_sensors[maxIndex + 1]:
                percentage = ir_sensors[maxIndex - 1] / ir_sensors[maxIndex]
                angle_between = left_angle - middle_angle
                angle = middle_angle + angle_between * percentage
            else:
                percentage = ir_sensors[maxIndex + 1] / ir_sensors[maxIndex]
                angle_between = middle_angle - right_angle
                angle = middle_angle - angle_between * percentage
        elif left_angle:
            percentage = ir_sensors[maxIndex - 1] / ir_sensors[maxIndex]
            angle_between = left_angle - middle_angle
            angle = middle_angle + angle_between * percentage
        elif right_angle:
            percentage = ir_sensors[maxIndex + 1] / ir_sensors[maxIndex]
            angle_between = middle_angle - right_angle
            angle = middle_angle - angle_between * percentage
    
    return angle / 130.6

def get_motion_lightring(ir_sensors: list[int], red: int = None, green: int = None, blue: int = None) -> list[_LedColor]:
    """Returns a list of LEDs that are highlighted based on IR sensor readings."""
    
    if len(ir_sensors) == 7:
        rotation = get_rotation_position(ir_sensors) # Returns percentage between 0.0 to 1.
        
        if red is not None and green is not None and blue is not None:
            led = _LedColor(red=red, green=green, blue=blue)
        else:
            led = lightring.get_hue_percentage(rotation)
        
        rotation = ((rotation * 130.6) + 180 - 65.3) / 360
        lightring_leds = []
        for i in range(6):
            lightring_leds += [lightring.adjust_rotation_brightness(led, rotation, constraints.get_led_angle(i))]
            
        return lightring_leds
    
    return None
