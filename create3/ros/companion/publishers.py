#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.models.companion import Publish

from .callbacks.handler import (
    publish_handler,
)

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class Publisher(Threading if TYPE_CHECKING else object):
    """Handles ROS publishers for companion data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Hidden global publish information
        self._publisher_msgs = Publish
        """Contains the most recent messages to be published for each topic. Updated when a set function is called."""

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        publisher_callback_group = MutuallyExclusiveCallbackGroup()
        publish_handler_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Publishers
        self._servo = self.node.create_publisher(Float32, 'servo_angle', qos_profile, callback_group=publisher_callback_group)

        self.node.create_timer(0.05, lambda: publish_handler(self), callback_group=publish_handler_callback_group)

        # Add topics to debugger
        self.debug.publishers = [self._servo]

    def servo_angle(self, angle: float | int):
        """Sets current angle of servo."""
        servo_msg = Float32()
        servo_msg.data = angle * 1.0 # Make sure angle is a float

        self._publisher_msgs.servo = servo_msg

    # def servo_angle(self, angle: float | int, speed: float | int = 171.43):
    #     """
    #     Sets current angle of servo.
        
    #     angle is between 50 deg and 180 deg, max speed of servo is 171.43 deg/s
    #     """
        
    #     servo = Servo()
    #     servo.angle = float(abs(angle))
    #     servo.angular_velocity = speed
        
    #     self._publish.servo = servo
        
    # def servo_adjust_angle(self, angle: float | int, speed: float | int = 171.43):
    #     """
    #     Adjust servo angle by specific degree. Angle in deg.
        
    #     max speed of servo is 171.43 deg/s
    #     """
        
    #     servo = Servo()
    #     servo.angle = float(abs(angle))
    #     servo.angular_velocity = speed
    #     servo.desired_angle = False
        
    #     self._publish.servo = servo
        
    # def servo_hold(self):
    #     """Holds last known angle position."""
        
    #     servo = Servo()
    #     servo.angle = self._subscribe.servo_angle
    #     servo.hold_angle = True
        
    #     self._publish.servo = servo
        
    # def servo_hold_stop(self):
    #     """Stops the hold on the servo position."""
        
    #     servo = Servo()
    #     servo.angle = self._subscribe.servo_angle
    #     servo.hold_angle = False
        
    #     self._publish.servo = servo
        
    # def set_first_led(self, brightness: float | int, blink_timing: float | int = 0.0):
    #     """Set the First LED brightness from 0 to 100 and set blink timing in seconds."""
        
    #     if 100 < brightness < 0:
    #         raise Exception("Error LED brightness needs to between 0 to 100")
        
    #     if blink_timing < 0:
    #         raise Exception("Error LED blink timing needs to be 0 or higher")
        
    #     led = Led()
    #     led.brightness = brightness
    #     led.blink_timing = blink_timing
        
    #     self._publish.leds.leds[0] = led
        
    # def set_second_led(self, brightness: float | int, blink_timing: float | int = 0.0):
    #     """Set the Second LED brightness from 0 to 100 and set blink timing in seconds."""
        
    #     if 100 < brightness < 0:
    #         raise Exception("Error LED brightness needs to between 0 to 100")
        
    #     if blink_timing < 0:
    #         raise Exception("Error LED blink timing needs to be 0 or higher")
        
    #     led = Led()
    #     led.brightness = brightness
    #     led.blink_timing = blink_timing
        
    #     self._publish.leds.leds[1] = led