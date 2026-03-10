#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector, LedColor

from .callbacks import HandlerCallbacks
from create3.utils import Threading

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class PublisherInterface(HandlerCallbacks, Threading if TYPE_CHECKING else object):
    """Handles ROS publishers for robot data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        cmd_velocity_callback_group = MutuallyExclusiveCallbackGroup()
        publisher_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Publishers
        self._lightring = self.node.create_publisher(LightringLeds, 'cmd_lightring', qos_profile, callback_group=publisher_callback_group)
        self._audio = self.node.create_publisher(AudioNoteVector, 'cmd_audio', qos_profile, callback_group=publisher_callback_group)
        self._velocities = self.node.create_publisher(Twist, 'cmd_vel', qos_profile, callback_group=cmd_velocity_callback_group)

        # Add topics to debugger
        self.debug.publishers = [self._lightring, self._audio, self._velocities]

    def set_lights_on_rgb(self, r: int, g: int, b: int):
        """Set all LEDs to the same RGB color. Values for r, g, b should be between 0 and 255."""
        
        # Set individual LED color
        led1 = LedColor(red=r, green=g, blue=b)
        led2 = LedColor(red=r, green=g, blue=b)
        led3 = LedColor(red=r, green=g, blue=b)
        led4 = LedColor(red=r, green=g, blue=b)
        led5 = LedColor(red=r, green=g, blue=b)
        led6 = LedColor(red=r, green=g, blue=b)
        
        # Create Lightring message
        led_msg = LightringLeds()
        led_msg.override_system = True
        led_msg.leds = [led1, led2, led3, led4, led5, led6]
        
        # Save locally so publish in background
        self._publisher_msgs.lightring = led_msg

    def set_lights(self, leds: list[LedColor]):
        """Set all LEDs to specified colors. List should be of length 6 with values for r, g, b between 0 and 255."""
        
        # Create Lightring message
        led_msg = LightringLeds()
        led_msg.override_system = True
        led_msg.leds = leds
        
        # Save locally so publish in background
        self._publisher_msgs.lightring = led_msg
        
    def set_lights_off(self):
        """Turn off all Lightring LEDs."""
        # Save locally so publish in background
        self._publisher_msgs.lightring = LightringLeds()
    
    def set_wheel_speeds(self, left_wheel: float | int, right_wheel: float | int):
        """Set wheel speeds in cm/s. Values should be between -46 and 46 cm/s."""
        
        # Calculate linear and angular speeds
        twist_msg = Twist()
        twist_msg.linear.x = ((right_wheel + left_wheel) / 100) / 2 # ( right_wheel(cm/s) + left_wheel(cm/s) ) / 100(convert to m/s) / 2 = linear_velocity(m/s)
        twist_msg.angular.z = (right_wheel - left_wheel) / self.tools.constraints.WHEEL_DISTANCE_APART # ( right_wheel(cm/s) - left_wheel(cm/s) ) / wheel_distance(cm) = angular_velocity(rad/s)
        
        # Set wheel speeds
        self._publisher_msgs.wheel_speeds = twist_msg
        
    def set_left_speed(self, speed: float | int):
        """Set Left Wheel speed individually. Wheel speed in cm/s"""
        
        # Find right wheel speeds from last change
        right_wheel = (self._publisher_msgs.wheel_speeds.linear.x * 100) + (self.tools.constraints.WHEEL_DISTANCE_APART * self._publisher_msgs.wheel_speeds.angular.z) / 2 # linear_velocity(cm/s) + wheel_distance(cm) * angular_velocity(rad/s) / 2 = right_wheel(cm/s)
        
        # set wheel speeds
        self.set_wheel_speeds(speed, right_wheel)
        
    def set_right_speed(self, speed: float | int):
        """Set Right Wheel speed individually. Wheel speed in cm/s"""
        
        # Find left wheel speeds from last change
        left_wheel = (self._publisher_msgs.wheel_speeds.linear.x * 100) - (self.tools.constraints.WHEEL_DISTANCE_APART * self._publisher_msgs.wheel_speeds.angular.z) / 2 # linear_velocity(cm/s) - wheel_distance(cm) * angular_velocity(rad/s) / 2 = left_wheel(cm/s)
        
        # set wheel speeds
        self.set_wheel_speeds(left_wheel, speed)