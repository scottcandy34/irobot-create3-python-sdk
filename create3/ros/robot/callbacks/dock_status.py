#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from irobot_create_msgs.msg import DockStatus

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def dock_status_callback(subscriber: "Subscriber", status: DockStatus) -> None:
    """Update docking-related values (visible, docked)."""
    subscriber.docking_values.dock_visible._update_state(status.dock_visible)
    subscriber.docking_values.is_docked._update_state(status.is_docked)
