#
# ROS Topic Publisher Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector, LedColor

from ..threading import RobotThreading, RpiThreading, PcThreading

pub_qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class RobotPublishers(RobotThreading if TYPE_CHECKING else object):
    """Publish to ROS topics by sending messages."""
    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Create Publishers
        self._lightring = self.node.create_publisher(LightringLeds, 'cmd_lightring', pub_qos_profile, callback_group=self._otherCbGroup)
        self._audio = self.node.create_publisher(AudioNoteVector, 'cmd_audio', pub_qos_profile, callback_group=self._otherCbGroup)
        self._velocities = self.node.create_publisher(Twist, 'cmd_vel', pub_qos_profile, callback_group=self._cmdVelCbGroup)
        
    def set_lights_on_rgb(self, r: int, g: int, b: int):
        """Set robot's LED to animation with color red, green, blue."""
        
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
        self._publish.lightring = led_msg

    def set_lights(self, leds: list[LedColor]):
        """Send list of LEDs to be published."""
        
        # Create Lightring message
        led_msg = LightringLeds()
        led_msg.override_system = True
        led_msg.leds = leds
        
        # Save locally so publish in background
        self._publish.lightring = led_msg
        
    def set_lights_off(self):
        """Will turn off any set lights."""
        # Save locally so publish in background
        self._publish.lightring = LightringLeds()
    
    def set_wheel_speeds(self, left_wheel: float | int, right_wheel: float | int):
        """Set Wheel speeds individually. Wheel speeds in cm/s"""
        
        # Calculate linear and angular speeds
        twist_msg = Twist()
        twist_msg.linear.x = ((right_wheel + left_wheel) / 100) / 2 # ( right_wheel(cm/s) + left_wheel(cm/s) ) / 100(convert to m/s) / 2 = linear_velocity(m/s)
        twist_msg.angular.z = (right_wheel - left_wheel) / self.tools.values.wheelDistanceApart # ( right_wheel(cm/s) - left_wheel(cm/s) ) / wheel_distance(cm) = angular_velocity(rad/s)
        
        # Set wheel speeds
        self._publish.wheel_speeds = twist_msg
        
    def set_left_speed(self, speed: float | int):
        """Set Left Wheel speed individually. Wheel speed in cm/s"""
        
        # Find right wheel speeds from last change
        right_wheel = (self._publish.wheel_speeds.linear.x * 100) + (self.tools.values.wheelDistanceApart * self._publish.wheel_speeds.angular.z) / 2 # linear_velocity(cm/s) + wheel_distance(cm) * angular_velocity(rad/s) / 2 = right_wheel(cm/s)
        
        # set wheel speeds
        self.set_wheel_speeds(speed, right_wheel)
        
    def set_right_speed(self, speed: float | int):
        """Set Right Wheel speed individually. Wheel speed in cm/s"""
        
        # Find left wheel speeds from last change
        left_wheel = (self._publish.wheel_speeds.linear.x * 100) - (self.tools.values.wheelDistanceApart * self._publish.wheel_speeds.angular.z) / 2 # linear_velocity(cm/s) - wheel_distance(cm) * angular_velocity(rad/s) / 2 = left_wheel(cm/s)
        
        # set wheel speeds
        self.set_wheel_speeds(left_wheel, speed)

    def _setWheelSpeedHandler(self):
        # Checks if wheelSpeeds object has been set or not then moves the robot with the message only lasts 0.5s
        if self._publish.wheel_speeds != Twist() and self._publish.wheel_speeds != self._publish.last_wheel_speeds:
            self._velocities.publish(self._publish.wheel_speeds)
        elif self._publish.wheel_speeds != Twist():
            self._velocities.publish(self._publish.wheel_speeds)
            
        self._publish.last_wheel_speeds = self._publish.wheel_speeds

    def _publishHandler(self):
        # Led Lightring Topic
        if self._publish.lightring != self._publish.last_lightring:
            self._lightring.publish(self._publish.lightring)

        self._publish.last_lightring = self._publish.lightring            
        
        # Audio Note Topic
        if self._publish.audio_note != self._publish.last_audio_note:
            self._audio.publish(self._publish.audio_note)
            
        self._publish.last_audio_note = self._publish.audio_note

class RpiPublishers(RpiThreading if TYPE_CHECKING else object):
    """Publish to ROS topics by sending messages."""
    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Create Publishers
        self._servo = self.node.create_publisher(Float32, 'servo_angle', pub_qos_profile, callback_group=self._otherCbGroup)

    def servo_angle(self, angle: float | int):
        servo_msg = Float32()
        servo_msg.data = angle * 1.0 # Make sure angle is a float

        self._publish.servo = servo_msg

    def _publishHandler(self):
        if self._publish.servo != self._publish.last_servo:
            self._servo.publish(self._publish.servo)
        
        self._publish.last_servo = self._publish.servo

class PcPublishers(PcThreading if TYPE_CHECKING else object):
    """Publish to ROS topics by sending messages."""
    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Create Publishers
        self._joy_feedback = self.node.create_publisher(JoyFeedbackArray, 'joy/set_feedback', pub_qos_profile, callback_group=self._otherCbGroup)

    def _publishHandler(self):
        if self._publish.rumble_enable and self._publish.rumble_running:
            feedback_array = JoyFeedbackArray()
            feedback = JoyFeedback()
            feedback.type = JoyFeedback.TYPE_RUMBLE
            feedback.id = 0  # find by  fftest /dev/input/event4

            def start():
                self._publish.rumble_running = True
                feedback.intensity = 1.0
                feedback_array.array = [feedback]
                self._joy_feedback.publish(feedback_array)

            def stop():
                self._publish.rumble_running = False
                feedback.intensity = 0.0
                feedback_array.array = [feedback]
                self._joy_feedback.publish(feedback_array)
                self._publish.rumble_running = False

            start()
            self.delay_callback(0.5, stop)
            self._publish.rumble_enable = False


    def controller_rumble(self):
        self._publish.rumble_enable = True