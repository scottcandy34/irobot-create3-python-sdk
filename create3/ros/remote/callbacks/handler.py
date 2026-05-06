#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.publisher import Publisher
from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback

if TYPE_CHECKING:
    from create3.ros.remote import Publisher

def publish_handler(publisher: "Publisher") -> None:
    """Handle controller rumble (vibration) feedback on the `/joy_feedback` topic.

    When `rumble_enable` is activated (and `rumble_running` is True), this
    function sends a short 0.5-second vibration pulse to the controller.
    It automatically stops the rumble after the pulse and clears the enable flag.

    Called every 0.05 seconds by the Publisher's timer.
    """
    if not (publisher.rumble_enable and publisher.rumble_running):
        return

    # Prepare reusable feedback message (rumble motor ID 0 is standard)
    feedback_array = JoyFeedbackArray()
    feedback = JoyFeedback()
    feedback.type = JoyFeedback.TYPE_RUMBLE
    feedback.id = 0

    def start_rumble() -> None:
        """Activate the controller rumble."""
        publisher.rumble_running = True
        feedback.intensity = 1.0
        feedback_array.array = [feedback]
        publisher.send_joy_feedback(feedback_array)

    def stop_rumble() -> None:
        """Stop the controller rumble after the pulse."""
        publisher.rumble_running = False
        feedback.intensity = 0.0
        feedback_array.array = [feedback]
        publisher.send_joy_feedback(feedback_array)

    # Trigger the one-shot rumble pulse
    start_rumble()
    publisher.node.create_oneshot_timer(0.5, stop_rumble)

    # Clear the enable flag so we don't retrigger on the next timer tick
    publisher.rumble_enable = False