#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JoyFeedbackArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.models.remote import Publish

from .callbacks.handler import (
    publish_handler
)

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class Publisher(Threading if TYPE_CHECKING else object):
    """ROS publisher for controller feedback (rumble/vibration).

    This lightweight Publisher is used by the remote/companion node to
    send rumble commands to the controller via the `/joy/set_feedback` topic.

    The background timer (0.05 s) calls the rumble handler, which manages
    the actual pulsing logic.
    """

    def __init__(self, node: Node) -> None:
        """Initialize the rumble feedback publisher and its background timer.

        Parameters
        ----------
        node : Node
            The ROS node that owns this publisher.
        """
        super().__init__(node)  # initialize Threading + Logger

        # Shared container that holds the latest messages to be published
        self._publisher_msgs: Publish = Publish()

        # Use a mutually exclusive callback group
        publisher_callback_group = MutuallyExclusiveCallbackGroup()
        publish_handler_callback_group = MutuallyExclusiveCallbackGroup()

        # Create the joy feedback publisher
        self._joy_feedback = self.node.create_publisher(JoyFeedbackArray, "joy/set_feedback", qos_profile, callback_group=publisher_callback_group)

        # Background timer that drives the rumble pulse logic
        self.node.create_timer(0.05, lambda: publish_handler(self), callback_group=publish_handler_callback_group)

        # Register with debugger for interface monitoring
        self.debug.publishers = [self._joy_feedback]

    def controller_rumble(self) -> None:
        """Trigger a short rumble pulse on the connected controller.

        The actual rumble (0.5-second vibration) is handled by the
        background `publish_handler` (rumble version).
        """
        self._publisher_msgs.rumble_enable = True