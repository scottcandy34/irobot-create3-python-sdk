#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from create3.ros.robot import Publisher

def publish_handler_callback(publisher: "Publisher") -> None:
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