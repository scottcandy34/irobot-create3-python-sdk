#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.node import Node
from sensor_msgs.msg import Joy
from tf2_ros.buffer import Buffer
from nav_msgs.msg import OccupancyGrid
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from yolo_msgs.msg import DetectionArray

from create3.utils import Threading
from create3.models.remote import Controller, Map, Yolo, Subscribe

from .callbacks.msg import (
    joy_callback,
    map_callback,
    yolo_detections_callback,
    corrected_position_callback
)

qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(Threading if TYPE_CHECKING else object):
    """ROS subscriber manager for companion/remote topics (joystick, map, YOLO).

    This class handles higher-level perception and control input topics.
    All callbacks run in a mutually exclusive callback group so they never
    interfere with each other or with other nodes.

    The class also registers itself with the debugger for uptime and
    interface monitoring.
    """

    def __init__(self, node: Node) -> None:
        """Initialize subscriptions for joystick input, occupancy grid, and YOLO detections.

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
        self._joy = self.node.create_subscription(Joy, 'joy', lambda msg: joy_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._map = self.node.create_subscription(OccupancyGrid, 'map', lambda msg: map_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._yolo_detections = self.node.create_subscription(DetectionArray, '/yolo/detections', lambda msg: yolo_detections_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)
        self.node.create_timer(0.05, corrected_position_callback)
        
        # Register all subscriptions with the debugger for uptime monitoring
        self.debug.subscriptions = [self._joy, self._map, self._yolo_detections]

    # ----------------------------------------------------------------------
    # Public getters (convenience API for the rest of the codebase)
    # ----------------------------------------------------------------------

    def get_controller(self) -> Controller:
        """Return the most recent controller (joystick) input data."""
        return self._subscription_msgs.controller

    def get_map(self) -> Map:
        """Return the most recent occupancy grid map data."""
        return self._subscription_msgs.map.data

    def get_yolo(self) -> Yolo:
        """Return the most recent YOLO object detections."""
        return self._subscription_msgs.yolo.data