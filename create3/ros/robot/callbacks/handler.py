#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist

if TYPE_CHECKING:
    from create3.ros.robot import Publisher

def set_wheel_speed_handler(publisher: "Publisher"):
    """Loop for setting wheel speeds Constantly every 0.5 sec if they have been updated."""
    if publisher._publisher_msgs.wheel_speeds != Twist() and publisher._publisher_msgs.wheel_speeds != publisher._publisher_msgs.last_wheel_speeds:
        publisher._velocities.publish(publisher._publisher_msgs.wheel_speeds)
    elif publisher._publisher_msgs.wheel_speeds != Twist():
        publisher._velocities.publish(publisher._publisher_msgs.wheel_speeds)
        
    publisher._publisher_msgs.last_wheel_speeds = publisher._publisher_msgs.wheel_speeds

def publish_handler(publisher: "Publisher"):
    """Loop for checking for updates and publishing Constantly every 0.5 sec for all topics except wheel speeds which has its own handler."""

    # Led Lightring Topic
    if publisher._publisher_msgs.lightring != publisher._publisher_msgs.last_lightring:
        publisher._lightring.publish(publisher._publisher_msgs.lightring)

    publisher._publisher_msgs.last_lightring = publisher._publisher_msgs.lightring            
    
    # Audio Note Topic
    if publisher._publisher_msgs.audio_note != publisher._publisher_msgs.last_audio_note:
        publisher._audio.publish(publisher._publisher_msgs.audio_note)
        
    publisher._publisher_msgs.last_audio_note = publisher._publisher_msgs.audio_note
