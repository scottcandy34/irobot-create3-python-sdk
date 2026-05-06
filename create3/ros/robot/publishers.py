#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from rclpy.node import Node
from rclpy.publisher import Publisher as Publishing
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector

from create3.utils import Logger
from create3.models.robot import Publish, Topics

from .callbacks.handler import (
    set_wheel_speed_handler,
    publish_handler,
)

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class Publisher(Logger):
    """ROS publisher manager for the iRobot Create3.

    Handles all outgoing robot commands:
      • Lightring LEDs
      • Audio notes
      • Wheel velocity (`cmd_vel`)

    Uses dedicated timers (0.05 s) and mutually exclusive callback groups
    so publishing never interferes with subscriptions or other callbacks.

    The `set_*` methods only update internal state — the background timers
    handle the actual publishing (continuous for wheel speeds, on-change
    for LEDs and audio).
    """

    def __init__(self, node: Node) -> None:
        """Initialize all publishers and background publish timers.

        Parameters
        ----------
        node : Node
            The ROS node that owns these publishers.
        """
        super().__init__(node)  # initialize Threading + Logger

        # Shared container that holds the latest messages to be published
        self.msgs: Publish = Publish()

        # Separate callback groups so different publish types never block each other
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Background timers (called every 50 ms)
        self.node.create_timer(0.05, lambda: set_wheel_speed_handler(self), callback_group=MutuallyExclusiveCallbackGroup())
        self.node.create_timer(0.05, lambda: publish_handler(self), callback_group=MutuallyExclusiveCallbackGroup())

        # Register publishers with the debugger for interface monitoring
        self.topics: list[Publishing] = []
        
    def find(self, name: Topics) -> Publishing:
        for publisher in self.topics:
            if name == publisher.topic_name:
                return publisher
            
        return None
    
    # =====================================================================
    # Public API — Audio Note commands
    # =====================================================================
    
    @property
    def audio_note(self) -> AudioNoteVector:
        return self.msgs.audio_note
    
    @audio_note.setter
    def audio_note(self, msg: AudioNoteVector):
        self.msgs.audio_note = msg
        
    @property
    def last_audio_note(self) -> AudioNoteVector:
        return self.msgs.last_audio_note
    
    @last_audio_note.setter
    def last_audio_note(self, msg: AudioNoteVector):
        self.msgs.last_audio_note = msg
    
    def send_audio(self, audio_note_msg: AudioNoteVector):
        if not self.find(Topics.CMD_AUDIO):
            self.topics.append(self.node.create_publisher(AudioNoteVector, Topics.CMD_AUDIO, qos_profile, callback_group=self.callback_group))
            
        self.find(Topics.CMD_AUDIO).publish(audio_note_msg)

    # =====================================================================
    # Public API — LED commands
    # =====================================================================
    
    @property
    def lightring(self) -> LightringLeds:
        return self.msgs.lightring
    
    @lightring.setter
    def lightring(self, msg: LightringLeds):
        self.msgs.lightring = msg
        
    @property
    def last_lightring(self) -> LightringLeds:
        return self.msgs.last_lightring
    
    @last_lightring.setter
    def last_lightring(self, msg: LightringLeds):
        self.msgs.last_lightring = msg
    
    def send_lightring(self, lightring_leds_msg: LightringLeds):
        if not self.find(Topics.CMD_LIGHTRING):
            self.topics.append(self.node.create_publisher(LightringLeds, Topics.CMD_LIGHTRING, qos_profile, callback_group=self.callback_group))
            
        self.find(Topics.CMD_LIGHTRING).publish(lightring_leds_msg)

    # =====================================================================
    # Public API — Wheel velocity commands
    # =====================================================================
    
    @property
    def velocity(self) -> Twist:
        return self.msgs.velocity
    
    @velocity.setter
    def velocity(self, msg: Twist):
        self.msgs.velocity = msg
        
    @property
    def last_velocity(self) -> Twist:
        return self.msgs.last_velocity
    
    @last_velocity.setter
    def last_velocity(self, msg: Twist):
        self.msgs.last_velocity = msg
    
    def send_velocity(self, twist_msg: Twist):
        if not self.find(Topics.CMD_VEL):
            self.topics.append(self.node.create_publisher(Twist, Topics.CMD_VEL, qos_profile, callback_group=MutuallyExclusiveCallbackGroup()))
            
        self.find(Topics.CMD_VEL).publish(twist_msg)