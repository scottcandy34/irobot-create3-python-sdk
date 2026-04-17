#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from geometry_msgs.msg import Twist
from irobot_create_msgs.action import LedAnimation
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector

from create3.models.common import Position
from .objects import HazardBumper, HazardCliff, RobotButtons, Acceleration, DockingValues

class Subscribe():
    """Holds all robot subscribed topics."""
    position = Position() # Current position
    ir_values = [0, 0, 0, 0, 0, 0, 0]
    bumpers = HazardBumper()
    cliff = HazardCliff()
    buttons = RobotButtons()
    battery: int | float = 100
    acceleration = Acceleration()
    dockingValues = DockingValues()
    
class Publish():
    """Holds all robot published topics."""
    wheel_speeds = Twist()
    last_wheel_speeds = Twist()
    
    lightring = LightringLeds()
    last_lightring = LightringLeds()
    
    led_animation = LedAnimation.Goal()
    last_led_animation = LedAnimation.Goal()
    
    audio_note = AudioNoteVector()
    last_audio_note = AudioNoteVector()