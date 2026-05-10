#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from rclpy.publisher import Publisher as Publishing
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JoyFeedbackArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Logger
from create3.models.remote import Publish, Topics

from .callbacks import (
    publish_handler_callback
)

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class Publisher(Logger):
    """ROS publisher for controller feedback (rumble/vibration).

    This lightweight Publisher is used by the remote/companion node to
    send rumble commands to the controller via the `/joy/set_feedback` topic.

    The background timer (0.05 s) calls the rumble handler, which manages
    the actual pulsing logic.
    """

    def __init__(self, *args, **kwargs) -> None:
        """Initialize the rumble feedback publisher and its background timer.

        Parameters
        ----------
        node : Node
            The ROS node that owns this publisher.
        """
        super().__init__(*args, **kwargs)

        # Shared container that holds the latest messages to be published
        self.msgs: Publish = Publish()

        # Use a mutually exclusive callback group
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Background timer that drives the rumble pulse logic
        self.node.create_timer(0.05, lambda: publish_handler_callback(self), callback_group=MutuallyExclusiveCallbackGroup())

        # Register publishers with the watchdog for interface monitoring
        self.topics: list[Publishing] = []
        
    def find(self, name: Topics) -> Publishing:
        for publisher in self.topics:
            if name == publisher.topic_name:
                return publisher
            
        return None
    
    @property
    def rumble_enable(self) -> bool:
        return self.msgs.rumble_enable
    
    @rumble_enable.setter
    def rumble_enable(self, msg: bool):
        self.msgs.rumble_enable = msg
        
    @property
    def rumble_running(self) -> bool:
        return self.msgs.rumble_running
    
    @rumble_running.setter
    def rumble_running(self, msg: bool):
        self.msgs.rumble_running = msg
    
    def send_joy_feedback(self, joy_feedback_msg: JoyFeedbackArray):
        if not self.find(Topics.JOY_FEEDBACK):
            self.topics.append(self.node.create_publisher(JoyFeedbackArray, Topics.JOY_FEEDBACK, qos_profile, callback_group=self.callback_group))
            
        self.find(Topics.JOY_FEEDBACK).publish(joy_feedback_msg)
