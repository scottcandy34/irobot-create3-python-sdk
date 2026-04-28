#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from geometry_msgs.msg import Twist
from dataclasses import dataclass, field
from irobot_create_msgs.action import LedAnimation
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector

from create3.models.common import Position, Stamped, TopicContainer
from .objects import HazardBumper, HazardCliff, RobotButtons, Acceleration, DockingValues

@dataclass
class Subscribe(TopicContainer):
    """Container holding the most recent data from all robot subscriptions.

    Updated automatically by the callbacks in the robot's `Subscriber` class.
    """

    # Odometry & pose
    position: Stamped[Position] = field(default_factory=lambda: Stamped(Position()))

    # IR proximity (7 sensors)
    ir_values: Stamped[list[int]] = field(default_factory=lambda: Stamped([0] * 7))

    # Hazard detection
    bumpers: HazardBumper = field(default_factory=HazardBumper)
    cliff: HazardCliff = field(default_factory=HazardCliff)

    # Physical buttons
    buttons: RobotButtons = field(default_factory=RobotButtons)

    # Battery & IMU
    battery: int | float = 100
    acceleration: Stamped[Acceleration] = field(default_factory=lambda: Stamped(Acceleration()))

    # Docking sensors
    dockingValues: DockingValues = field(default_factory=DockingValues)

@dataclass
class Publish(TopicContainer):
    """Container holding the current and previous messages for all robot publishers.

    Used by the background publish handlers to decide when to send new commands.
    """

    # Wheel velocity (continuously published)
    wheel_speeds: Twist = field(default_factory=Twist)
    last_wheel_speeds: Twist = field(default_factory=Twist)

    # Lightring LEDs
    lightring: LightringLeds = field(default_factory=LightringLeds)
    last_lightring: LightringLeds = field(default_factory=LightringLeds)

    # LED animation goal (action)
    led_animation: LedAnimation.Goal = field(default_factory=LedAnimation.Goal)
    last_led_animation: LedAnimation.Goal = field(default_factory=LedAnimation.Goal)

    # Audio note sequence (action)
    audio_note: AudioNoteVector = field(default_factory=AudioNoteVector)
    last_audio_note: AudioNoteVector = field(default_factory=AudioNoteVector)