from typing import TYPE_CHECKING

from .subscribers import Subscriber
from .publishers import Publisher
from create3.utils import Node, Threading, remote as tools
from create3.models.remote import Controller, Map, Yolo

class Interface(Threading if TYPE_CHECKING else object):
    """Mixin that exposes all user-facing methods for the RemoteNode."""
    def __init__(self, node: Node):
        super().__init__(node)  # initialize Threading + Logger
        
        # Create internal components
        self.subscriber = Subscriber(node)
        self.publisher = Publisher(node)
        self.actions = None
        self.services = None
        
    def is_alive(self) -> list[tuple[str, bool]]:
        """Return a list of all ROS interfaces belonging to this device.

        Format: list of `(interface_name, True)` tuples.
        Used by the Watchdog to track which interfaces are present.
        """
        subs = [(sub.topic_name, True) for sub in self.subscriber.topics]
        pubs = [(pub.topic_name, True) for pub in self.publisher.topics]

        return subs + pubs
        
    # ===================================================================
    # SUBSCRIBER GETTERS
    # ===================================================================

    def get_controller(self) -> Controller:
        """Return the most recent controller (joystick) input data."""
        return self.subscriber.controller

    def get_map(self) -> Map:
        """Return the most recent occupancy grid map data."""
        return self.subscriber.map.data

    def get_yolo(self) -> Yolo:
        """Return the most recent YOLO object detections."""
        return self.subscriber.yolo.data
    
    # ===================================================================
    # PUBLISHER COMMANDS
    # ===================================================================

    def controller_rumble(self) -> None:
        """Trigger a short rumble pulse on the connected controller.

        The actual rumble (0.5-second vibration) is handled by the
        background `publish_handler` (rumble version).
        """
        self.publisher.rumble_enable = True