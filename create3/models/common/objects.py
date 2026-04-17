#
# Common Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

@dataclass
class Position:
    """Stores position values."""
    x: int | float = 0.0
    y: int | float = 0.0
    angle: int | float = 0.0