#
# Object Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#
import pprint

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector
from irobot_create_msgs.action import LedAnimation

TIMEOUT = 0.8 # timeout for action servers
DEFAULT_WAIT = 3 # delay in receiving command
ROUNDING_VALUE = 6

class Position():
    """Stores robot position values."""
    x: int | float = 0.0
    y: int | float = 0.0
    angle: int | float = 0.0

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)

class HazardBumper():
    """Stores robot bumper values."""
    right: bool = False
    front_right: bool = False
    front_center: bool = False
    front_left: bool = False
    left: bool = False

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)
    
class HazardCliff():
    """Stores robot cliff sensor values."""
    side_right: bool = False
    front_right: bool = False
    front_left: bool = False
    side_left: bool = False

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)
    
class Acceleration():
    """Stores robot acceleration values."""
    x: int | float = 0.0
    y: int | float = 0.0
    z: int | float = 0.0

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)
    
class DockingValues():
    """Stores robot docking values."""
    is_docked = False
    dock_visible = False
    sensor: int = 0
    greenBuoy = False
    redBuoy = False
    forceField = False

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)
    
class Buttons():
    """Stores robot button pressed values."""
    button_1 = False
    button_power = False
    button_2 = False

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)
    
class RobotSubscribeTopics():
    """Holds all robot subscribed topics."""
    position = Position() # Current position
    ir_values = [0, 0, 0, 0, 0, 0, 0]
    bumpers = HazardBumper()
    cliff = HazardCliff()
    buttons = Buttons()
    battery: int | float = 100
    acceleration = Acceleration()
    dockingValues = DockingValues()
    
class RobotPublishTopics():
    """Holds all robot published topics."""
    wheel_speeds = Twist()
    last_wheel_speeds = Twist()
    
    lightring = LightringLeds()
    last_lightring = LightringLeds()
    
    led_animation = LedAnimation.Goal()
    last_led_animation = LedAnimation.Goal()
    
    audio_note = AudioNoteVector()
    last_audio_note = AudioNoteVector()