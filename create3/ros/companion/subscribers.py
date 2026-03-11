#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from sensor_msgs.msg import LaserScan, Range
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

from create3.utils import Threading
from .callbacks import MessageHandler

sub_qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(MessageHandler, Threading if TYPE_CHECKING else object):
    """Handles ROS subscribers for companion data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Subscription
        self._scan = self.node.create_subscription(LaserScan, 'scan', self._scan_callback, sub_qos_profile, callback_group=subscriber_callback_group)
        self._range = self.node.create_subscription(Range, 'range', self._range_callback, sub_qos_profile, callback_group=subscriber_callback_group)

        # Add topics to debugger
        self.debug.subscriptions = [self._scan, self._range]