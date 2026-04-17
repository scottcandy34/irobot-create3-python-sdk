#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from create3.ros.companion import Publisher

def publish_handler(publisher: "Publisher"):
    if publisher._publisher_msgs.servo != publisher._publisher_msgs.last_servo:
        publisher._servo.publish(publisher._publisher_msgs.servo)
    
    publisher._publisher_msgs.last_servo = publisher._publisher_msgs.servo