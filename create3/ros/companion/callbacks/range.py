#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.time import Time
from sensor_msgs.msg import Range

from create3.models.common import Stamped
from create3.models.companion import Ultrasonic

if TYPE_CHECKING:
    from create3.ros.companion import Subscriber

def range_callback(subscriber: "Subscriber", range_: Range) -> None:
    """Handle incoming ultrasonic Range data and update the shared ultrasonic state.

    Converts all distance values from meters to centimeters to match the
    internal units used throughout the SDK.
    """
    ultrasonic = Ultrasonic()

    ultrasonic.field_of_view = range_.field_of_view

    # Convert distances to cm
    ultrasonic.max_range = range_.max_range * 100.0
    ultrasonic.min_range = range_.min_range * 100.0
    ultrasonic.range = range_.range * 100.0
    
    subscriber.ultrasonic = Stamped(ultrasonic, Time.from_msg(range_.header.stamp))
    