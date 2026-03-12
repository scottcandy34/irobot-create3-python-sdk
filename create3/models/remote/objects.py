#
# Remote Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import pprint as _pprint

class Joystick():
    horizontal: float = 0.0
    vertical: float = 0.0
    button: bool = False

class Dpad():
    left: bool = False
    right: bool = False
    up: bool = False
    down: bool = False

class JoyButtons():
    x: bool = False
    circle: bool = False
    triangle: bool = False
    square: bool = False
    l1: bool = False
    r1: bool = False
    share: bool = False
    options: bool = False
    ps: bool = False

class Controller():
    """Stores ps controller button pressed values."""
    left_joy = Joystick()
    left_trigger: float = 0.0
    right_joy = Joystick()
    right_trigger: float = 0.0
    dpad = Dpad()
    buttons = JoyButtons()

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)