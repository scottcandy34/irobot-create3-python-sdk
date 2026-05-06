import math
import time
from typing import TYPE_CHECKING

from std_msgs.msg import Float32

from .subscribers import Subscriber
from .publishers import Publisher
from create3.utils import Node, Threading, companion as tools

class Interface(Threading if TYPE_CHECKING else object):
    """Mixin that exposes all user-facing methods for the RemoteNode."""
    def __init__(self, node: Node):
        super().__init__(node)  # initialize Threading + Logger
        
        # Create internal components
        self.subscriber = Subscriber(node)
        self.publisher = Publisher(node)
        self.actions = None
        self.services = None
        
    def is_alive(self) -> list[tuple[str, bool]]:
        """Return a list of all ROS interfaces belonging to this device.

        Format: list of `(interface_name, True)` tuples.
        Used by the Debugger to track which interfaces are present.
        """
        subs = [(sub.topic_name, True) for sub in self.subscriber.topics]
        pubs = [(pub.topic_name, True) for pub in self.publisher.topics]

        return subs + pubs
        
    # ===================================================================
    # SUBSCRIBER GETTERS
    # ===================================================================

    def get_scans(self) -> list[float]:
        """Return the most recent LiDAR scan ranges (in centimeters)."""
        return self.subscriber.lidar.data.ranges

    def get_range(self) -> float:
        """Return the most recent ultrasonic range measurement (in centimeters)."""
        return self.subscriber.ultrasonic.data.range
    
    # ===================================================================
    # PUBLISHER COMMANDS
    # ===================================================================

    def reset_servo(self) -> None:
        """Reset the servo to the default 90° (center) position.

        Blocks for 1 second to allow the physical servo to reach the position.
        """
        servo_msg = Float32()
        servo_msg.data = 90.0

        self.publisher.send_servo_angle(servo_msg)
        self.publisher.servo = servo_msg

        time.sleep(1.0)  # give the servo time to physically move

    def set_servo_angle(self, angle: float | int) -> None:
        """Set the servo to an absolute angle (degrees)."""
        servo_msg = Float32()
        servo_msg.data = float(angle)

        self.publisher.servo = servo_msg

    def set_servo_angle_with_speed(self, target_angle: float | int, speed: float | int) -> None:
        """Smoothly move the servo to `target_angle` (degrees) at constant speed (rad/s).

        Uses the validated servo tools to ensure safe limits and produces a
        smooth ramp by sending incremental position commands at 50 Hz.
        """
        target = tools.servo.validate_angle(target_angle)
        speed = tools.servo.validate_speed(speed)  # always positive

        # Current position (default to 90° if no previous command)
        current = getattr(self.publisher.last_servo, "data", 90.0)

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