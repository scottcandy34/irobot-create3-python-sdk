#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from sensor_msgs.msg import Joy
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

from .callbacks import MessageHandler
from create3.utils import Threading
from create3.models.remote import Controller, Map

qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(MessageHandler, Threading if TYPE_CHECKING else object):
    """Handles ROS subscribers for robot data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Subscription
        self._joy = self.node.create_subscription(Joy, 'joy', self._joy_callback, qos_profile, callback_group=subscriber_callback_group)
        self._map = self.node.create_subscription(OccupancyGrid, 'map', self._map_callback, qos_profile, callback_group=subscriber_callback_group)

        # Add topics to debugger
        self.debug.subscriptions = [self._joy, self._map]

    def get_controller(self) -> Controller:
        """Returns the controller input."""
        return self._subscription_msgs.controller
    
    def get_map(self) -> Map:
        """Returns the latest map message."""
        return self._subscription_msgs.map