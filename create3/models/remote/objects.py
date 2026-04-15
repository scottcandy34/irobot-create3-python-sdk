#
# Remote Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

@dataclass
class Joystick():
    horizontal: float = 0.0
    vertical: float = 0.0
    button: bool = False

@dataclass
class Dpad():
    left: bool = False
    right: bool = False
    up: bool = False
    down: bool = False

@dataclass
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

@dataclass
class Controller():
    """Stores ps controller button pressed values."""
    left_joy = Joystick()
    left_trigger: float = 0.0
    right_joy = Joystick()
    right_trigger: float = 0.0
    dpad = Dpad()
    buttons = JoyButtons()