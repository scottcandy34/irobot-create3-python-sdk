#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from sensor_msgs.msg import Joy
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from yolo_msgs.msg import DetectionArray

from create3.utils import Threading
from create3.models.remote import Controller, Map, Yolo, Subscribe

from .callbacks.msg import (
    joy_callback,
    map_callback,
    yolo_detections_callback
)

qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(Threading if TYPE_CHECKING else object):
    """Handles ROS subscribers for robot data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Hidden global callback information
        self._subscription_msgs = Subscribe
        """Contains the most recent messages received for each topic. Updated when a callback is triggered."""

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Subscription
        self._joy = self.node.create_subscription(Joy, 'joy', lambda msg: joy_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._map = self.node.create_subscription(OccupancyGrid, 'map', lambda msg: map_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._yolo_detections = self.node.create_subscription(DetectionArray, '/yolo/detections', lambda msg: yolo_detections_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)

        # Add topics to debugger
        self.debug.subscriptions = [self._joy, self._map, self._yolo_detections]

    def get_controller(self) -> Controller:
        """Returns the controller input."""
        return self._subscription_msgs.controller
    
    def get_map(self) -> Map:
        """Returns the latest map message."""
        return self._subscription_msgs.map

    def get_yolo(self) -> Yolo:
        """Returns the latest YOLO detections."""
        return self._subscription_msgs.yolo