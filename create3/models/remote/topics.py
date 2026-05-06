#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from enum import StrEnum, auto
from dataclasses import dataclass, field

from .objects import Controller, Map, Yolo
from create3.models.common import Stamped, TopicContainer

class Topics(StrEnum):
    def _generate_next_value_(name, start, count, last_values):
        # Add a prefix to the auto-generated name
        return f"/{name.lower()}"
    
    JOY = auto()
    JOY_FEEDBACK = "/joy/set_feedback"
    MAP = auto()
    YOLO_DETECTIONS = "/yolo/detections"

@dataclass
class Subscribe(TopicContainer):
    """Container holding the most recent data from all remote subscriptions.

    Updated automatically by the callbacks in the remote node's `Subscriber` class.
    """

    # Joystick / controller input
    controller: Controller = field(default_factory=Controller)

    # Occupancy grid map
    map: Stamped[Map] = field(default_factory=lambda: Stamped(Map()))

    # YOLO object detections
    yolo: Stamped[Yolo] = field(default_factory=lambda: Stamped(Yolo()))

@dataclass
class Publish(TopicContainer):
    """Container holding the current state of all remote publishers.

    Used by the background publish handlers to decide when to send commands.
    """

    # Controller rumble (vibration) flag
    rumble_enable: bool = False
    rumble_running: bool = False