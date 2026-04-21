#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from sensor_msgs.msg import LaserScan, Range
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

from create3.utils import Threading
from create3.models.companion import Subscribe

from .callbacks.msg import (
    scan_callback,
    range_callback
)

qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(Threading if TYPE_CHECKING else object):
    """Handles ROS subscribers for companion data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Hidden global callback information
        self._subscription_msgs = Subscribe
        """Contains the most recent messages received for each topic. Updated when a callback is triggered."""

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Subscription
        self._scan = self.node.create_subscription(LaserScan, 'scan', lambda msg: scan_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._range = self.node.create_subscription(Range, 'range', lambda msg: range_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)

        # Add topics to debugger
        self.debug.subscriptions = [self._scan, self._range]

    def get_scans(self) -> list[float]:
        """Returns the most recent lidar scan ranges."""
        return self._subscription_msgs.lidar.ranges
    
    def get_range(self) -> float:
        """Returns the most recent ultrasonic range."""
        return self._subscription_msgs.ultrasonic.range