#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import LedAnimation
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector

from .objects import Position, HazardBumper, HazardCliff, Acceleration, DockingValues, RobotButtons, Lidar, Ultrasonic, Controller

class _robot_subscribe_topics():
    """Holds all robot subscribed topics."""
    position = Position() # Current position
    ir_values = [0, 0, 0, 0, 0, 0, 0]
    bumpers = HazardBumper()
    cliff = HazardCliff()
    buttons = RobotButtons()
    battery: int | float = 100
    acceleration = Acceleration()
    dockingValues = DockingValues()
    
class _robot_publish_topics():
    """Holds all robot published topics."""
    wheel_speeds = Twist()
    last_wheel_speeds = Twist()
    
    lightring = LightringLeds()
    last_lightring = LightringLeds()
    
    led_animation = LedAnimation.Goal()
    last_led_animation = LedAnimation.Goal()
    
    audio_note = AudioNoteVector()
    last_audio_note = AudioNoteVector()

class _rpi_subscribe_topics():
    """Holds all rpi subscribed topics."""
    lidar = Lidar()
    ultrasonic = Ultrasonic()
    servo_angle = 90.0
    
class _rpi_publish_topics():
    """Holds all rpi published topics."""
    servo = Float32()
    last_servo = Float32()

class _pc_subscribe_topics():
    """Holds all pc subscribed topics."""
    controller = Controller()

class _pc_publish_topics():
    """Holds all pc published topics."""
    rumble_enable: bool = False
    rumble_running: bool = False

class SubscriberTopics():
    robot = _robot_subscribe_topics()
    rpi = _rpi_subscribe_topics()
    pc = _pc_subscribe_topics()

class PublisherTopics():
    robot = _robot_publish_topics()
    rpi = _rpi_publish_topics()
    pc = _pc_publish_topics()