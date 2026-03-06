#
# Object Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#
import pprint

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector
from irobot_create_msgs.action import LedAnimation
from std_msgs.msg import Float32

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

class Lidar():
    """Stores rpi lidar values."""
    angle_min: float = 0.0 # start angle of scan
    angle_max: float = 0.0 # end angle of scan
    angle_increment: float = 0.0 # angular distance between measurements
    range_min: float = 0.0 # minimum range value
    range_max: float = 0.0 # maximum range value
    time_increment: float = 0.0 # rime between measurements
    scan_time: float = 0.0 # time between scans
    ranges: list[float] = []

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)

class Ultrasonic():
    """Stores rpi ultrasonic sensor values."""
    field_of_view: float = 0.0
    min_range: float = 0.0
    max_range: float = 0.0
    range: float = 0.0

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)

class RpiSubscribeTopics():
    """Holds all rpi subscribed topics."""
    lidar = Lidar()
    ultrasonic = Ultrasonic()
    servo_angle = 90.0
    
class RpiPublishTopics():
    """Holds all rpi published topics."""
    servo = Float32()
    last_servo = Float32()
    
class Joystick():
    horizontal: float = 0.0
    vertical: float = 0.0
    button: bool = False

class Dpad():
    left: bool = False
    right: bool = False
    up: bool = False
    down: bool = False

class Buttons():
    x: bool = False
    circle: bool = False
    triangle: bool = False
    square: bool = False
    l1: bool = False
    r1: bool = False
    share: bool = False
    options: bool = False
    ps: bool = False

class Controller():
    """Stores ps controller button pressed values."""
    left_joy = Joystick()
    left_trigger: float = 0.0
    right_joy = Joystick()
    right_trigger: float = 0.0
    dpad = Dpad()
    buttons = Buttons()

    def __str__(self):
        return pprint.pformat(self, indent = 4, width = 80)

class PcSubscribeTopics():
    """Holds all pc subscribed topics."""
    controller = Controller()

class PcPublishTopics():
    """Holds all pc published topics."""
    rumble_enable: bool = False
    rumble_running: bool = False