#
# IR Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with the IR sensors on the iRobot Create3."""

from irobot_create_msgs.msg import LedColor as _LedColor

from . import lightring, constraints

def get_rotation_position(ir_sensors: list[int]) -> float:
    """Estimate the precise rotation of the strongest IR signal as a normalized percentage (0.0–1.0).

    Uses the 7 IR sensors to find the brightest one, then performs linear
    interpolation with its immediate neighbors for sub-sensor angular precision.
    The result is normalized by the total sensor field-of-view span (130.6°).

    Parameters
    ----------
    ir_sensors : list[int]
        List of 7 raw IR sensor readings (higher = stronger signal).

    Returns
    -------
    float
        Normalized rotation: 0.0–1.0 (or 0.0 if no signal or invalid input).
    """
    if len(ir_sensors) != 7 or not ir_sensors:
        return 0.0

    # Find the strongest sensor
    strongest_index = ir_sensors.index(max(ir_sensors))
    strongest_value = ir_sensors[strongest_index]

    if strongest_value <= 0:
        return 0.0

    # Get angles for this sensor and its neighbors (may be None at edges)
    left_angle = constraints.get_ir_angle(strongest_index - 1)
    middle_angle = constraints.get_ir_angle(strongest_index)
    right_angle = constraints.get_ir_angle(strongest_index + 1)

    angle = 0.0

    if left_angle is not None and right_angle is not None:
        # Interpolate using the stronger neighbor
        if ir_sensors[strongest_index - 1] > ir_sensors[strongest_index + 1]:
            percentage = ir_sensors[strongest_index - 1] / strongest_value
            angle_between = left_angle - middle_angle
            angle = middle_angle + angle_between * percentage
        else:
            percentage = ir_sensors[strongest_index + 1] / strongest_value
            angle_between = middle_angle - right_angle
            angle = middle_angle - angle_between * percentage

    elif left_angle is not None:
        percentage = ir_sensors[strongest_index - 1] / strongest_value
        angle_between = left_angle - middle_angle
        angle = middle_angle + angle_between * percentage

    elif right_angle is not None:
        percentage = ir_sensors[strongest_index + 1] / strongest_value
        angle_between = middle_angle - right_angle
        angle = middle_angle - angle_between * percentage

    # Normalize to 0.0–1.0 (total sensor span = 130.6°)
    return angle / 130.6

def get_motion_lightring(ir_sensors: list[int], red: int | None = None, green: int | None = None, blue: int | None = None) -> list[_LedColor] | None:
    """Generate a 6-LED lightring pattern that highlights the direction of the strongest IR signal.

    The LEDs are lit with a brightness "spotlight" effect centered on the
    detected IR rotation. Color is either user-specified RGB or a hue wheel
    based on the rotation percentage.

    Parameters
    ----------
    ir_sensors : list[int]
        List of 7 raw IR sensor readings.
    red, green, blue : int | None
        Optional fixed RGB color (0–255). If any is None, a hue-based color
        is used instead.

    Returns
    -------
    list[_LedColor] | None
        List of 6 LED colors for the lightring, or None if ir_sensors is not length 7.
    """
    if len(ir_sensors) != 7:
        return None

    # Get normalized rotation from IR sensors (0.0–1.0)
    rotation = get_rotation_position(ir_sensors)

    # Choose base LED color
    if red is not None and green is not None and blue is not None:
        led = _LedColor(red=red, green=green, blue=blue)
    else:
        led = lightring.get_hue_percentage(rotation)

    # Offset and normalize rotation to match the physical LED ring layout
    # (maps the 130.6° IR field into the 360° LED circle with centering)
    led_rotation = ((rotation * 130.6) + 180.0 - 65.3) / 360.0

    # Build the lightring pattern
    lightring_leds: list[_LedColor] = []
    for i in range(6):
        led_color = lightring.adjust_rotation_brightness(led, led_rotation, constraints.get_led_angle(i))
        lightring_leds.append(led_color)

    return lightring_leds