#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector, LedColor

from create3.utils import Threading
import create3.utils.robot as tools
from create3.models.robot import Publish

from .callbacks.handler import (
    set_wheel_speed_handler,
    publish_handler,
)

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=1
)

class Publisher(Threading if TYPE_CHECKING else object):
    """ROS publisher manager for the iRobot Create3.

    Handles all outgoing robot commands:
      • Lightring LEDs
      • Audio notes
      • Wheel velocity (`cmd_vel`)

    Uses dedicated timers (0.05 s) and mutually exclusive callback groups
    so publishing never interferes with subscriptions or other callbacks.

    The `set_*` methods only update internal state — the background timers
    handle the actual publishing (continuous for wheel speeds, on-change
    for LEDs and audio).
    """

    def __init__(self, node: Node) -> None:
        """Initialize all publishers and background publish timers.

        Parameters
        ----------
        node : Node
            The ROS node that owns these publishers.
        """
        super().__init__(node)  # initialize Threading + Logger

        # Shared container that holds the latest messages to be published
        self._publisher_msgs: Publish = Publish()

        # Separate callback groups so different publish types never block each other
        cmd_velocity_callback_group = MutuallyExclusiveCallbackGroup()
        publisher_callback_group = MutuallyExclusiveCallbackGroup()
        publish_handler_callback_group = MutuallyExclusiveCallbackGroup()
        set_wheel_speed_callback_group = MutuallyExclusiveCallbackGroup()

        # Create publishers
        self._lightring = self.node.create_publisher(LightringLeds, 'cmd_lightring', qos_profile, callback_group=publisher_callback_group)
        self._audio = self.node.create_publisher(AudioNoteVector, 'cmd_audio', qos_profile, callback_group=publisher_callback_group)
        self._velocities = self.node.create_publisher(Twist, 'cmd_vel', qos_profile, callback_group=cmd_velocity_callback_group)

        # Background timers (called every 50 ms)
        self.node.create_timer(0.05, lambda: set_wheel_speed_handler(self), callback_group=set_wheel_speed_callback_group)
        self.node.create_timer(0.05, lambda: publish_handler(self), callback_group=publish_handler_callback_group)

        # Register publishers with the debugger for interface monitoring
        self.debug.publishers = [self._lightring, self._audio, self._velocities]

    # =====================================================================
    # Public API — LED commands
    # =====================================================================

    def set_lights_on_rgb(self, r: int, g: int, b: int) -> None:
        """Set all six LEDs to the same RGB color.

        Values for r, g, b must be in the range 0–255.
        """
        led = LedColor(red=r, green=g, blue=b)
        led_msg = LightringLeds()
        led_msg.override_system = True
        led_msg.leds = [led] * 6

        self._publisher_msgs.lightring = led_msg

    def set_lights(self, leds: list[LedColor]) -> None:
        """Set each of the six LEDs to a custom color.

        `leds` must be a list of exactly 6 `LedColor` objects.
        """
        led_msg = LightringLeds()
        led_msg.override_system = True
        led_msg.leds = leds

        self._publisher_msgs.lightring = led_msg

    def set_lights_off(self) -> None:
        """Turn off all Lightring LEDs."""
        self._publisher_msgs.lightring = LightringLeds()

    # =====================================================================
    # Public API — Wheel velocity commands
    # =====================================================================

    def set_wheel_speeds(self, left_wheel: float | int, right_wheel: float | int) -> None:
        """Set both wheel speeds in cm/s (range approximately -46 to +46 cm/s).

        This is the primary way to drive the robot. The background timer
        will continuously publish the command.
        """
        twist = Twist()
        # Convert cm/s to m/s and compute differential-drive kinematics
        twist.linear.x = ((right_wheel + left_wheel) / 100.0) / 2.0
        twist.angular.z = (right_wheel - left_wheel) / tools.constraints.WHEEL_DISTANCE_APART

        self._publisher_msgs.wheel_speeds = twist

    def set_left_speed(self, speed: float | int) -> None:
        """Set only the left wheel speed in cm/s (right wheel is kept from last command)."""
        if self._publisher_msgs.wheel_speeds == Twist():
            right_wheel = 0.0
        else:
            right_wheel = ((self._publisher_msgs.wheel_speeds.linear.x * 100.0) + (tools.constraints.WHEEL_DISTANCE_APART * self._publisher_msgs.wheel_speeds.angular.z) / 2.0)
        self.set_wheel_speeds(speed, right_wheel)

    def set_right_speed(self, speed: float | int) -> None:
        """Set only the right wheel speed in cm/s (left wheel is kept from last command)."""
        if self._publisher_msgs.wheel_speeds == Twist():
            left_wheel = 0.0
        else:
            left_wheel = ((self._publisher_msgs.wheel_speeds.linear.x * 100.0) - (tools.constraints.WHEEL_DISTANCE_APART * self._publisher_msgs.wheel_speeds.angular.z) / 2.0)
        self.set_wheel_speeds(left_wheel, speed)

    def send_twist(self, twist_msg: Twist) -> None:
        """Immediately publish a Twist command (bypasses the 0.5 s continuous handler).

        Useful for one-shot velocity commands.
        """
        self._velocities.publish(twist_msg)