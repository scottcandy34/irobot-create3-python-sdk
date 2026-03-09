#
# Lightring Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with the lightring on the iRobot Create3, including adjusting brightness and color based on rotation and generating colors based on percentage."""

import colorsys as _colorsys

from irobot_create_msgs.msg import LedColor as _LedColor

def adjust_brightness(led: _LedColor, brightness: float) -> _LedColor:
    """Return LED color based on brightness percentage between 0.0 and 1.0"""
    
    new_led = _LedColor()
    new_led.red = int(led.red * brightness)
    new_led.green = int(led.green * brightness)
    new_led.blue = int(led.blue * brightness)
    
    return new_led
    
def adjust_rotation_brightness(led: _LedColor, rotation: float, led_degree: float, span: float = 90) -> _LedColor:
    """Return LED color based on rotation and led degree with a certain span in degrees for brightness falloff"""

    position = rotation * 360
    
    x1 = led_degree
    if led_degree > position and (position - 90) < (led_degree - 360):
        x1 = led_degree - 360
        
    x2 = led_degree
    if led_degree < position and (position) < (led_degree + 360) < (position + 90):
        x2 = led_degree + 360
    
    z1 = (x1 - (position - span)) / span # return percentage within range span
    z2 = ((position + span) - x2) / span # return percentage within range span
    
    brightness = 0.0
    if 0 < z1 < 1.0:
        brightness = z1
    elif 0 < z2 < 1.0:
        brightness = z2
    elif z1 == 1.0 or z2 == 1.0:
        brightness = 1.0
        
    return adjust_brightness(led, brightness)

def get_hue_percentage(percentage: float, start_hue: int = 0, end_hue: int = 360) -> _LedColor:
    """Return LED color based on percentage between 0.0 and 1.0 with a certain hue range in degrees"""
    
    hue = ((percentage * abs(end_hue - start_hue)) + start_hue) / 360
    lightness = 0.5
    saturation = 1.0
    
    colors = _colorsys.hls_to_rgb(hue, lightness, saturation)
    
    led = _LedColor()
    led.red = int(colors[0] * 255)
    led.green = int(colors[1] * 255)
    led.blue = int(colors[2] * 255)
    
    return led
