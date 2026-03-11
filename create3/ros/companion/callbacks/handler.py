#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.models import PublisherTopics

class HandlerCallbacks(Threading if TYPE_CHECKING else object):
    """Handles the publishing of messages to topics."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Hidden global publish information
        self._publisher_msgs = PublisherTopics.Companion
        """Contains the most recent messages to be published for each topic. Updated when a set function is called."""

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        publish_handler_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.node.create_timer(0.05, self._publish_handler, callback_group=publish_handler_callback_group)

    def _publish_handler(self):
        if self._publisher_msgs.servo != self._publisher_msgs.last_servo:
            self._servo.publish(self._publisher_msgs.servo)
        
        self._publisher_msgs.last_servo = self._publisher_msgs.servo