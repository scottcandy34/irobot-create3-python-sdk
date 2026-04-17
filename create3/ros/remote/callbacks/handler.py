#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.publisher import Publisher
from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback

if TYPE_CHECKING:
    from create3.ros.remote import Publisher

def publish_handler(publisher: "Publisher"):
    if publisher._publisher_msgs.rumble_enable and publisher._publisher_msgs.rumble_running:
        feedback_array = JoyFeedbackArray()
        feedback = JoyFeedback()
        feedback.type = JoyFeedback.TYPE_RUMBLE
        feedback.id = 0  # find by  fftest /dev/input/event4

        def start():
            publisher._publisher_msgs.rumble_running = True
            feedback.intensity = 1.0
            feedback_array.array = [feedback]
            publisher._joy_feedback.publish(feedback_array)

        def stop():
            publisher._publisher_msgs.rumble_running = False
            feedback.intensity = 0.0
            feedback_array.array = [feedback]
            publisher._joy_feedback.publish(feedback_array)
            publisher._publisher_msgs.rumble_running = False

        start()
        publisher.delay_callback(0.5, stop)
        publisher._publisher_msgs.rumble_enable = False
