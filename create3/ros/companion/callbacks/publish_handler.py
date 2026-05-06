#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from create3.ros.companion import Publisher

def publish_handler_callback(publisher: "Publisher") -> None:
    """Periodically publish servo commands when they change.

    This handler is called every 0.05 seconds by the Publisher class timer.
    It follows the same "publish-only-on-change" pattern used for lightring
    and audio to reduce unnecessary network traffic.
    """
    current_servo = publisher.servo

    # Publish only if the servo command has changed
    if current_servo != publisher.last_servo:
        publisher.send_servo_angle(current_servo)

    # Always update the last-known value for the next cycle
    publisher.last_servo = current_servo