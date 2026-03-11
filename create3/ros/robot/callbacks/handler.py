#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.models import PublisherTopics

class PublishHandler(Threading if TYPE_CHECKING else object):
    """Handles the publishing of messages to topics."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Hidden global publish information
        self._publisher_msgs = PublisherTopics.ROBOT
        """Contains the most recent messages to be published for each topic. Updated when a set function is called."""

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        publish_handler_callback_group = MutuallyExclusiveCallbackGroup()
        set_wheel_speed_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.node.create_timer(0.05, self._publish_handler, callback_group=publish_handler_callback_group)

        # Creates a timer that will loop every 0.499s and set wheel speeds if exist
        node.create_timer(0.05, self._set_wheel_speed_handler, callback_group=set_wheel_speed_callback_group)

    def _set_wheel_speed_handler(self):
        """Loop for setting wheel speeds Constantly every 0.5 sec if they have been updated."""
        if self._publisher_msgs.wheel_speeds != Twist() and self._publisher_msgs.wheel_speeds != self._publisher_msgs.last_wheel_speeds:
            self._velocities.publish(self._publisher_msgs.wheel_speeds)
        elif self._publisher_msgs.wheel_speeds != Twist():
            self._velocities.publish(self._publisher_msgs.wheel_speeds)
            
        self._publisher_msgs.last_wheel_speeds = self._publisher_msgs.wheel_speeds

    def _publish_handler(self):
        """Loop for checking for updates and publishing Constantly every 0.5 sec for all topics except wheel speeds which has its own handler."""

        # Led Lightring Topic
        if self._publisher_msgs.lightring != self._publisher_msgs.last_lightring:
            self._lightring.publish(self._publisher_msgs.lightring)

        self._publisher_msgs.last_lightring = self._publisher_msgs.lightring            
        
        # Audio Note Topic
        if self._publisher_msgs.audio_note != self._publisher_msgs.last_audio_note:
            self._audio.publish(self._publisher_msgs.audio_note)
            
        self._publisher_msgs.last_audio_note = self._publisher_msgs.audio_note
