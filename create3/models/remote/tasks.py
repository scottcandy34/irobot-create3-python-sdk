#
# Remote Tasks for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from enum import StrEnum, auto

class Tasks(StrEnum):
    """Tasks that run on the remote control node."""

    CONTROLLER = auto()
    """Read joystick input from the remote and translate it into robot movement and actions."""
