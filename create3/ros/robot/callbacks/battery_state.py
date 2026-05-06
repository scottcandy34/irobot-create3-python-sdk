#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from sensor_msgs.msg import BatteryState

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def battery_state_callback(subscriber: "Subscriber", battery: BatteryState) -> None:
    """Update battery percentage (converted to 0–100 scale) and issue a warning when low."""
    subscriber.battery = battery.percentage * 100.0

    if subscriber.battery <= 10.0:
        subscriber.print_warning(f"Battery low: {subscriber.msgs.battery:.1f}% remaining.")
