#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from sensor_msgs.msg import Joy

if TYPE_CHECKING:
    from create3.ros.remote import Subscriber

def joy_callback(subscriber: "Subscriber", joy: Joy) -> None:
    """Handle incoming joystick (Joy) data and update the shared controller state.

    Maps the raw axes and buttons from a standard PlayStation-style controller
    into the custom `Controller` object used throughout the codebase.
    """
    axes = joy.axes
    buttons = joy.buttons

    controller = subscriber.controller

    # Left stick + triggers
    controller.left_joy.horizontal = axes[0]
    controller.left_joy.vertical = axes[1]
    controller.left_trigger = axes[2]

    # Right stick + triggers
    controller.right_joy.horizontal = axes[3]
    controller.right_joy.vertical = axes[4]
    controller.right_trigger = axes[5]

    # D-pad (treated as axes on many controllers)
    controller.dpad.left._update_state(axes[6] > 0)
    controller.dpad.right._update_state(axes[6] < 0)
    controller.dpad.up._update_state(axes[7] > 0)
    controller.dpad.down._update_state(axes[7] < 0)

    # Face buttons + shoulder + special buttons
    controller.buttons.x._update_state(buttons[0] == 1)
    controller.buttons.circle._update_state(buttons[1] == 1)
    controller.buttons.triangle._update_state(buttons[2] == 1)
    controller.buttons.square._update_state(buttons[3] == 1)
    controller.buttons.l1._update_state(buttons[4] == 1)
    controller.buttons.r1._update_state(buttons[5] == 1)
    controller.buttons.share._update_state(buttons[8] == 1)
    controller.buttons.options._update_state(buttons[9] == 1)
    controller.buttons.ps._update_state(buttons[10] == 1)

    # Stick press buttons
    controller.left_joy.button._update_state(buttons[11] == 1)
    controller.right_joy.button._update_state(buttons[12] == 1)
