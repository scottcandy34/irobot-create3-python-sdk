#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from std_msgs.msg import Float32

from .objects import Lidar, Ultrasonic, DetectedShapes

class Subscribe():
    """Holds all companion subscribed topics."""
    lidar = Lidar()
    ultrasonic = Ultrasonic()
    servo_angle = 90.0
    
class Publish():
    """Holds all companion published topics."""
    servo = Float32()
    last_servo = Float32()