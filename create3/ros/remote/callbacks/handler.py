#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.models import PublisherTopics

class PublishHandler(Threading if TYPE_CHECKING else object):
    """Handles the publishing of messages to topics."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Hidden global publish information
        self._publisher_msgs = PublisherTopics.Remote
        """Contains the most recent messages to be published for each topic. Updated when a set function is called."""

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        publish_handler_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.node.create_timer(0.05, self._publish_handler, callback_group=publish_handler_callback_group)

    def _publish_handler(self):
        if self._publisher_msgs.rumble_enable and self._publisher_msgs.rumble_running:
            feedback_array = JoyFeedbackArray()
            feedback = JoyFeedback()
            feedback.type = JoyFeedback.TYPE_RUMBLE
            feedback.id = 0  # find by  fftest /dev/input/event4

            def start():
                self._publisher_msgs.rumble_running = True
                feedback.intensity = 1.0
                feedback_array.array = [feedback]
                self._joy_feedback.publish(feedback_array)

            def stop():
                self._publisher_msgs.rumble_running = False
                feedback.intensity = 0.0
                feedback_array.array = [feedback]
                self._joy_feedback.publish(feedback_array)
                self._publisher_msgs.rumble_running = False

            start()
            self.delay_callback(0.5, stop)
            self._publisher_msgs.rumble_enable = False
