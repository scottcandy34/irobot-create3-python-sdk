#
# Remote Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import numpy as np
from dataclasses import dataclass, field

from create3.models.common import Position, Button

@dataclass
class Joystick:
    """Stores one analog joystick axis and its press button."""

    horizontal: float = 0.0
    vertical: float = 0.0
    button: Button = field(default_factory=Button)

@dataclass
class Dpad:
    """Stores the four directional pad states."""

    left: Button = field(default_factory=Button)
    right: Button = field(default_factory=Button)
    up: Button = field(default_factory=Button)
    down: Button = field(default_factory=Button)

@dataclass
class JoyButtons:
    """Stores all face, shoulder, and special buttons on a PlayStation-style controller."""

    x: Button = field(default_factory=Button)
    circle: Button = field(default_factory=Button)
    triangle: Button = field(default_factory=Button)
    square: Button = field(default_factory=Button)
    l1: Button = field(default_factory=Button)
    r1: Button = field(default_factory=Button)
    share: Button = field(default_factory=Button)
    options: Button = field(default_factory=Button)
    ps: Button = field(default_factory=Button)

@dataclass
class Controller:
    """Stores the complete state of a PlayStation-style controller."""

    left_joy: Joystick = field(default_factory=Joystick)
    left_trigger: float = 0.0
    right_joy: Joystick = field(default_factory=Joystick)
    right_trigger: float = 0.0
    dpad: Dpad = field(default_factory=Dpad)
    buttons: JoyButtons = field(default_factory=JoyButtons)

@dataclass
class Map:
    """Stores the latest occupancy grid map received from the robot."""

    resolution: float = 0.0
    origin: Position = field(default_factory=Position)
    grid: np.ndarray = field(default_factory=lambda: np.empty((0, 0)))

@dataclass
class BoundingBox:
    """Stores a single object detection from YOLO."""

    class_id: int = 0
    class_name: str = ""
    score: float = 0.0
    tracking_id: int = 0
    center_x: float = 0.0
    center_y: float = 0.0
    width: float = 0.0
    height: float = 0.0
    theta: float = 0.0

@dataclass
class Yolo:
    """Stores all current YOLO detections."""

    bounding_boxes: list[BoundingBox] = field(default_factory=list)
    