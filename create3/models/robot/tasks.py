#
# Robot Tasks for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from enum import StrEnum, auto

class Tasks(StrEnum):
    """Tasks that run on the main robot node."""

    IR_LIGHTRING = auto()
    """Use IR proximity sensors to create a directional lightring pattern on the robot's LEDs."""
