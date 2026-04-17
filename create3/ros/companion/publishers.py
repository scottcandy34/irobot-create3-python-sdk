#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.models.companion import Publish

from .callbacks.handler import (
    publish_handler,
)

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class Publisher(Threading if TYPE_CHECKING else object):
    """Handles ROS publishers for companion data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Hidden global publish information
        self._publisher_msgs = Publish
        """Contains the most recent messages to be published for each topic. Updated when a set function is called."""

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        publisher_callback_group = MutuallyExclusiveCallbackGroup()
        publish_handler_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Publishers
        self._servo = self.node.create_publisher(Float32, 'servo_angle', qos_profile, callback_group=publisher_callback_group)

        self.node.create_timer(0.05, lambda: publish_handler(self), callback_group=publish_handler_callback_group)

        # Add topics to debugger
        self.debug.publishers = [self._servo]

    def servo_angle(self, angle: float | int):
        """Sets current angle of servo."""
        servo_msg = Float32()
        servo_msg.data = angle * 1.0 # Make sure angle is a float

        self._publisher_msgs.servo = servo_msg