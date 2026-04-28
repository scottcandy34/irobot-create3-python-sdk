#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.node import Node
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
    """ROS subscriber manager for companion data (LiDAR and ultrasonic).

    Handles the `/scan` (LaserScan) and `/range` (Range) topics.
    All callbacks run inside a mutually exclusive callback group so they never
    block each other. The class also registers itself with the debugger for
    uptime and interface monitoring.
    """

    def __init__(self, node: Node) -> None:
        """Initialize subscriptions for LiDAR scan and ultrasonic range data.

        Parameters
        ----------
        node : Node
            The ROS node that owns these subscriptions.
        """
        super().__init__(node)  # initialize Threading + Logger

        # Shared container that holds the latest message data for every topic
        self._subscription_msgs: Subscribe = Subscribe()

        # Use a mutually exclusive callback group so callbacks never block each other
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        # Create subscriptions
        self._scan = self.node.create_subscription(LaserScan, "scan", lambda msg: scan_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._range = self.node.create_subscription(Range, "range", lambda msg: range_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)

        # Register subscriptions with the debugger for uptime monitoring
        self.debug.subscriptions = [self._scan, self._range]

    # ----------------------------------------------------------------------
    # Public getters (convenience API for the rest of the codebase)
    # ----------------------------------------------------------------------

    def get_scans(self) -> list[float]:
        """Return the most recent LiDAR scan ranges (in centimeters)."""
        return self._subscription_msgs.lidar.data.ranges

    def get_range(self) -> float:
        """Return the most recent ultrasonic range measurement (in centimeters)."""
        return self._subscription_msgs.ultrasonic.data.range