#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

from .objects import Controller, Map, Yolo

@dataclass
class Subscribe:
    """Container holding the most recent data from all remote subscriptions.

    Updated automatically by the callbacks in the remote node's `Subscriber` class.
    """

    # Joystick / controller input
    controller: Controller = field(default_factory=Controller)

    # Occupancy grid map
    map: Map = field(default_factory=Map)

    # YOLO object detections
    yolo: Yolo = field(default_factory=Yolo)

@dataclass
class Publish:
    """Container holding the current state of all remote publishers.

    Used by the background publish handlers to decide when to send commands.
    """

    # Controller rumble (vibration) flag
    rumble_enable: bool = False