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
    current = publisher.velocity

    # Publish on change OR if we have an active non-zero movement command
    if (current != publisher.last_velocity) or (current != Twist()):
        publisher.send_velocity(current)

    # Always update the last-known command for next comparison
    publisher.last_velocity = current

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
    current_lightring = publisher.lightring
    if current_lightring != publisher.last_lightring:
        publisher.send_lightring(current_lightring)
    publisher.last_lightring = current_lightring

    # === Audio Note ===
    current_audio = publisher.audio_note
    if current_audio != publisher.last_audio_note:
        publisher.send_audio(current_audio)
    publisher.last_audio_note = current_audio