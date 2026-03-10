#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JoyFeedbackArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from .callbacks import HandlerCallbacks

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class PublisherInterface(HandlerCallbacks, Threading if TYPE_CHECKING else object):
    """Handles ROS publishers for robot data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        publisher_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Publishers
        self._joy_feedback = self.node.create_publisher(JoyFeedbackArray, 'joy/set_feedback', qos_profile, callback_group=publisher_callback_group)

        # Add topics to debugger
        self.debug.publishers = [self._joy_feedback]

    def controller_rumble(self):
        """Rumbles the controller if supported."""
        self._publisher_msgs.rumble_enable = True