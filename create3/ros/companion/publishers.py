#
# Publisher Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
import time
from typing import TYPE_CHECKING

from rclpy.node import Node
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
    """ROS publisher for companion servo control.

    Handles the `/servo_angle` topic used to command a pan/tilt servo
    attached to the companion node (e.g. camera mount).

    Uses a background timer (0.05 s) that calls the general `publish_handler`
    so servo commands are published only when they change.
    """

    tools: Tools

    def __init__(self, node: Node) -> None:
        """Initialize the servo publisher and background publish timer.

        Parameters
        ----------
        node : Node
            The ROS node that owns this publisher.
        """
        super().__init__(node)  # initialize Threading + Logger

        # Shared container that holds the latest messages to be published
        self._publisher_msgs: Publish = Publish()

        # Use a mutually exclusive callback group
        publisher_callback_group = MutuallyExclusiveCallbackGroup()
        publish_handler_callback_group = MutuallyExclusiveCallbackGroup()

        # Create the servo angle publisher
        self._servo = self.node.create_publisher(Float32, "servo_angle", qos_profile, callback_group=publisher_callback_group)

        # Background timer that drives the "publish only on change" logic
        self.node.create_timer(0.05, lambda: publish_handler(self), callback_group=publish_handler_callback_group)

        # Register with debugger for interface monitoring
        self.debug.publishers = [self._servo]

        # Move servo to default position on startup
        self.reset_servo()

    def reset_servo(self) -> None:
        """Reset the servo to the default 90° (center) position.

        Blocks for 1 second to allow the physical servo to reach the position.
        """
        servo_msg = Float32()
        servo_msg.data = 90.0

        self._servo.publish(servo_msg)
        self._publisher_msgs.servo = servo_msg

        time.sleep(1.0)  # give the servo time to physically move

    def set_servo_angle(self, angle: float | int) -> None:
        """Set the servo to an absolute angle (degrees)."""
        servo_msg = Float32()
        servo_msg.data = float(angle)

        self._publisher_msgs.servo = servo_msg

    def set_servo_angle_with_speed(self, target_angle: float | int, speed: float | int) -> None:
        """Smoothly move the servo to `target_angle` (degrees) at constant speed (rad/s).

        Uses the validated servo tools to ensure safe limits and produces a
        smooth ramp by sending incremental position commands at 50 Hz.
        """
        target = self.tools.servo.validate_angle(target_angle)
        speed = self.tools.servo.validate_speed(speed)  # always positive

        # Current position (default to 90° if no previous command)
        current = getattr(self._publisher_msgs.last_servo, "data", 90.0)

        angle_diff = abs(target - current)
        if angle_diff < 0.1:  # already at target
            self.set_servo_angle(target)
            return

        direction = 1 if target > current else -1
        desired_deg_per_s = math.degrees(speed)
        dt = 1.0 / 50.0  # 50 Hz update rate
        total_time = angle_diff / desired_deg_per_s
        num_steps = max(1, round(total_time / dt))
        step_size = (angle_diff / num_steps) * direction

        for _ in range(num_steps):
            current += step_size
            self.set_servo_angle(current)
            time.sleep(dt)

        # Final snap to exact target
        self.set_servo_angle(target)