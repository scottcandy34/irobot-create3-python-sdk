#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist

if TYPE_CHECKING:
    from create3.ros.robot import Publisher

def set_wheel_speed_handler(publisher: "Publisher") -> None:
    """Periodically publish wheel velocity commands (/cmd_vel) every 0.05 seconds.

    This handler ensures continuous robot motion by:
      • Publishing immediately when the wheel speed command changes
      • Continuously re-publishing any non-empty (non-zero) command every cycle
        (standard ROS practice to prevent velocity timeout/safety stop)

    Zero commands (default Twist()) are not re-published.
    """
    current = publisher._publisher_msgs.wheel_speeds

    # Publish on change OR if we have an active non-zero movement command
    if (current != publisher._publisher_msgs.last_wheel_speeds) or (current != Twist()):
        publisher._velocities.publish(current)

    # Always update the last-known command for next comparison
    publisher._publisher_msgs.last_wheel_speeds = current

def publish_handler(publisher: "Publisher") -> None:
    """Periodically check for changes and publish to non-velocity topics every 0.05 seconds.

    This is the general publish handler used by the Publisher class. It currently
    handles:
      • Lightring LEDs
      • Audio notes

    Uses a lightweight "publish only when changed" pattern to reduce unnecessary
    network traffic for these topics.
    """
    # === Lightring LEDs ===
    current_lightring = publisher._publisher_msgs.lightring
    if current_lightring != publisher._publisher_msgs.last_lightring:
        publisher._lightring.publish(current_lightring)
    publisher._publisher_msgs.last_lightring = current_lightring

    # === Audio Note ===
    current_audio = publisher._publisher_msgs.audio_note
    if current_audio != publisher._publisher_msgs.last_audio_note:
        publisher._audio.publish(current_audio)
    publisher._publisher_msgs.last_audio_note = current_audio