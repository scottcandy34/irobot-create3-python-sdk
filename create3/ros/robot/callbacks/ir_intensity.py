#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.time import Time
from irobot_create_msgs.msg import IrIntensityVector

from create3.models.common import Stamped

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def ir_intensity_callback(subscriber: "Subscriber", ir: IrIntensityVector) -> None:
    """Handle IR intensity readings and store the 7 sensor values in the shared state."""
    readings = ir.readings
    ir_values = [
        readings[0].value,
        readings[1].value,
        readings[2].value,
        readings[3].value,
        readings[4].value,
        readings[5].value,
        readings[6].value,
    ]
    
    subscriber.ir_values = Stamped(ir_values, Time.from_msg(ir.header.stamp))
