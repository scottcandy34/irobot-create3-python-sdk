#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from irobot_create_msgs.msg import InterfaceButtons

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def interface_buttons_callback(subscriber: "Subscriber", buttons: InterfaceButtons) -> None:
    """Update the state of the physical buttons on the robot."""
    subscriber.buttons.button_1._update_state(buttons.button_1.is_pressed)
    subscriber.buttons.button_power._update_state(buttons.button_power.is_pressed)
    subscriber.buttons.button_2._update_state(buttons.button_2.is_pressed)
