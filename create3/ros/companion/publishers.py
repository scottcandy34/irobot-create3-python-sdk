#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
import time
from typing import TYPE_CHECKING

from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.utils import companion as Tools
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

    tools: Tools

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

        self.reset_servo()

    def reset_servo(self):
        """Resets the servo to the default position."""
        servo_msg = Float32()
        servo_msg.data = 90.0
        self._servo.publish(servo_msg)
        self._publisher_msgs.servo = servo_msg
        time.sleep(1) # Give time for the servo to reset before any new commands are sent

    def set_servo_angle(self, angle: float | int):
        """Sets current angle of servo."""
        servo_msg = Float32()
        servo_msg.data = angle * 1.0 # Make sure angle is a float

        self._publisher_msgs.servo = servo_msg

    def set_servo_angle_with_speed(self, target_angle: float | int, speed: float | int):
        """Move to target_angle at constant speed (rad/s)."""
        target = self.tools.servo.validate_angle(target_angle)
        speed = self.tools.servo.validate_speed(speed)                     # now always positive

        current = self._publisher_msgs.last_servo.data
        angle_diff = abs(target - current)
        if angle_diff < 0.1:                              # already there
            self.set_servo_angle(target)
            return

        direction = 1 if target > current else -1
        desired_deg_per_s = math.degrees(speed)
        dt = 1.0 / 50.0                                   # 50 Hz update rate
        total_time = angle_diff / desired_deg_per_s
        num_steps = max(1, round(total_time / dt))        # how many position updates
        step_size = (angle_diff / num_steps) * direction  # degrees per update

        for _ in range(num_steps):
            current += step_size
            self.set_servo_angle(current)
            time.sleep(dt)

        self.set_servo_angle(target)                      # final snap to exact target