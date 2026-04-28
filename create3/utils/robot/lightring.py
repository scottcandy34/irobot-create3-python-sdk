#
# Lightring Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with the lightring on the iRobot Create3, including adjusting brightness and color based on rotation and generating colors based on percentage."""

import math
import colorsys as _colorsys

from irobot_create_msgs.msg import LedColor as _LedColor

def adjust_brightness(led: _LedColor, brightness: float) -> _LedColor:
    """Scale an LED color by a brightness factor (0.0 to 1.0).

    This is the standard way to dim an RGB LED while preserving its hue.

    Parameters
    ----------
    led : _LedColor
        Original LED color (red, green, blue).
    brightness : float
        Brightness multiplier. Will be clamped to the range [0.0, 1.0].

    Returns
    -------
    _LedColor
        New color with each channel scaled by the clamped brightness.
    """
    # Clamp brightness to valid range
    brightness = max(0.0, min(1.0, brightness))

    new_led = _LedColor()
    new_led.red = int(led.red * brightness)
    new_led.green = int(led.green * brightness)
    new_led.blue = int(led.blue * brightness)

    return new_led

def adjust_rotation_brightness(led: _LedColor, rotation: float, led_degree: float, span: float = 90.0) -> _LedColor:
    """Calculate a brightness falloff for an LED based on current rotation.

    Creates a "spotlight" effect: the LED is brightest when its angular
    position is inside the rotation ± span, and fades linearly outside it.
    Handles full 360° wrapping correctly.

    Parameters
    ----------
    led : _LedColor
        Original LED color.
    rotation : float
        Current rotation value (0.0–1.0 maps to 0°–360°).
    led_degree : float
        Angular position of this LED (in degrees).
    span : float
        Width of the brightness falloff zone in degrees (default 90).

    Returns
    -------
    _LedColor
        LED color scaled by the computed brightness (0.0–1.0).
    """
    current_position = rotation * 360.0  # convert normalized rotation to degrees

    # Handle angle wrapping so we always find the shortest distance
    x1 = led_degree
    if led_degree > current_position and (current_position - span) < (led_degree - 360):
        x1 = led_degree - 360

    x2 = led_degree
    if led_degree < current_position and current_position < (led_degree + 360) < (current_position + span):
        x2 = led_degree + 360

    # Normalized distance from the center of the spotlight
    z1 = (x1 - (current_position - span)) / span
    z2 = ((current_position + span) - x2) / span

    # Determine brightness (0.0 outside the span, 1.0 at the exact center)
    brightness = 0.0
    if 0.0 < z1 < 1.0:
        brightness = z1
    elif 0.0 < z2 < 1.0:
        brightness = z2
    elif math.isclose(z1, 1.0) or math.isclose(z2, 1.0):
        brightness = 1.0

    return adjust_brightness(led, brightness)

def get_hue_percentage(percentage: float, start_hue: int = 0, end_hue: int = 360) -> _LedColor:
    """Convert a percentage (0.0–1.0) into a fully saturated LED color in the given hue range.

    Uses HLS color space (hue, lightness=0.5, saturation=1.0) which produces
    vivid rainbow-style colors — perfect for LED status indicators.

    Parameters
    ----------
    percentage : float
        Value from 0.0 to 1.0. Will be clamped to this range.
    start_hue : int
        Starting hue in degrees (default 0).
    end_hue : int
        Ending hue in degrees (default 360).

    Returns
    -------
    _LedColor
        RGB LED color corresponding to the interpolated hue.
    """
    # Clamp percentage
    percentage = max(0.0, min(1.0, percentage))

    # Interpolate hue and normalize to [0, 1] for colorsys
    hue_range = abs(end_hue - start_hue)
    hue = ((percentage * hue_range) + start_hue) / 360.0

    lightness = 0.5
    saturation = 1.0

    # Convert HLS → RGB
    rgb = _colorsys.hls_to_rgb(hue, lightness, saturation)

    led = _LedColor()
    led.red = int(rgb[0] * 255)
    led.green = int(rgb[1] * 255)
    led.blue = int(rgb[2] * 255)

    return led