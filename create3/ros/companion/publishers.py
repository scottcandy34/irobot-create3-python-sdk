#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.publisher import Publisher as Publishing
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Logger, companion as Tools
from create3.models.companion import Publish, Topics

from .callbacks import (
    publish_handler_callback,
)

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class Publisher(Logger):
    """ROS publisher for companion servo control.

    Handles the `/servo_angle` topic used to command a pan/tilt servo
    attached to the companion node (e.g. camera mount).

    Uses a background timer (0.05 s) that calls the general `publish_handler`
    so servo commands are published only when they change.
    """

    tools: Tools

    def __init__(self, node: Node) -> None:
        """Initialize the servo publisher and background publish timer.

        Parameters
        ----------
        node : Node
            The ROS node that owns this publisher.
        """
        super().__init__(node)  # initialize Threading + Logger

        # Shared container that holds the latest messages to be published
        self.msgs: Publish = Publish()

        # Use a mutually exclusive callback group
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Background timer that drives the "publish only on change" logic
        self.node.create_timer(0.05, lambda: publish_handler_callback(self), callback_group=MutuallyExclusiveCallbackGroup())

        # Register publishers with the watchdog for interface monitoring
        self.topics: list[Publishing] = []
        
    def find(self, name: Topics) -> Publishing:
        for publisher in self.topics:
            if name == publisher.topic_name:
                return publisher
            
        return None
    
    @property
    def servo(self) -> Float32:
        return self.msgs.servo
    
    @servo.setter
    def servo(self, msg: Float32):
        self.msgs.servo = msg
        
    @property
    def last_servo(self) -> Float32:
        return self.msgs.last_servo
    
    @last_servo.setter
    def last_servo(self, msg: Float32):
        self.msgs.last_servo = msg
        
    def send_servo_angle(self, servo_angle_msg: Float32):
        if not self.find(Topics.SERVO_ANGLE):
            self.topics.append(self.node.create_publisher(Float32, Topics.SERVO_ANGLE, qos_profile, callback_group=self.callback_group))
            
        self.find(Topics.SERVO_ANGLE).publish(servo_angle_msg)
