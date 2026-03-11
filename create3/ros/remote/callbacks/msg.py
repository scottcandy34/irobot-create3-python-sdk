#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from sensor_msgs.msg import Joy

from create3.utils import Threading
from create3.models import SubscriberTopics, Controller

class MessageHandler(Threading if TYPE_CHECKING else object):
    """Handles callback functions for remote subscriptions."""

    def __init__(self, node):
        super().__init__(node)

        # Hidden global callback information
        self._subscription_msgs = SubscriberTopics.Remote
        """Contains the most recent messages received for each topic. Updated when a callback is triggered."""

    def _joy_callback(self, joy: Joy):
        self.update_uptime(self._joy.topic_name)

        joy_axes = joy.axes
        joy_buttons = joy.buttons

        controller = Controller()
        
        controller.left_joy.horizontal = joy_axes[0]
        controller.left_joy.vertical = joy_axes[1]
        controller.left_trigger = joy_axes[2]
        controller.right_joy.horizontal = joy_axes[3]
        controller.right_joy.vertical = joy_axes[4]
        controller.right_trigger = joy_axes[5]
        controller.dpad.left = joy_axes[6] > 0
        controller.dpad.right = joy_axes[6] < 0
        controller.dpad.up = joy_axes[7] > 0
        controller.dpad.down = joy_axes[7] < 0

        controller.buttons.x = joy_buttons[0] == 1
        controller.buttons.circle = joy_buttons[1] == 1
        controller.buttons.triangle = joy_buttons[2] == 1
        controller.buttons.square = joy_buttons[3] == 1
        controller.buttons.l1 = joy_buttons[4] == 1
        controller.buttons.r1 = joy_buttons[5] == 1
        # button 6 and 7 are also the left and right triggers but since it gets info from axes. its not used
        controller.buttons.share = joy_buttons[8] == 1
        controller.buttons.options = joy_buttons[9] == 1
        controller.buttons.ps = joy_buttons[10] == 1
        controller.left_joy.button = joy_buttons[11] == 1
        controller.right_joy.button = joy_buttons[12] == 1

        self._subscription_msgs.controller = controller