#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from enum import StrEnum, auto
from dataclasses import dataclass, field

from std_msgs.msg import Float32

from .objects import Lidar, Ultrasonic
from create3.models.common import Stamped, TopicContainer

class Topics(StrEnum):
    SCAN = auto()
    RANGE = auto()
    SERVO_ANGLE = auto()

@dataclass
class Subscribe(TopicContainer):
    """Container holding the most recent data from all companion subscriptions.

    Updated automatically by the callbacks in the companion node's `Subscriber` class.
    """

    # LiDAR and ultrasonic sensor data
    lidar: Stamped[Lidar] = field(default_factory=lambda: Stamped(Lidar()))
    ultrasonic: Stamped[Ultrasonic] = field(default_factory=lambda: Stamped(Ultrasonic()))

@dataclass
class Publish(TopicContainer):
    """Container holding the current state of all companion publishers.

    Used by the background publish handlers to decide when to send commands.
    """

    # Servo angle command
    servo: Float32 = field(default_factory=Float32)
    last_servo: Float32 = field(default_factory=Float32)