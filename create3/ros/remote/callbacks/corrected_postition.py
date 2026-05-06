#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.time import Time

if TYPE_CHECKING:
    from create3.ros.remote import Subscriber

def corrected_position_callback(subscriber: "Subscriber"):
    now = Time()
    trans = subscriber._tf_buffer.lookup_transform('map', 'base_link', now)
    
    x = trans.transform.translation.x
    y = trans.transform.translation.y
