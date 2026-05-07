import math
import time

from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from irobot_create_msgs.srv import ResetPose
from irobot_create_msgs.msg import LightringLeds, AudioNoteVector, LedColor, AudioNote
from irobot_create_msgs.action import NavigateToPosition, DriveArc, DriveDistance, RotateAngle, Dock, Undock, LedAnimation, AudioNoteSequence

from .subscribers import Subscriber
from .publishers import Publisher
from .actions import ActionClient
from .services import ServiceClient
from create3.utils import Threading, Node, robot as tools
from create3.utils.common.other import DEFAULT_WAIT
from create3.utils.common.coords import convert_to_quaternion, find_direction

class InterfaceMixin(Threading):
    """Mixin that exposes all user-facing methods for the RobotNode."""
    def __init__(self, node: Node):
        super().__init__(node)  # initialize Threading + Logger
        
        # Create internal components
        self.subscriber = Subscriber(node)
        self.publisher = Publisher(node)
        self.actions = ActionClient(node)
        self.services = ServiceClient(node)
        
    def is_alive(self) -> list[tuple[str, bool]]:
        """Return a list of all ROS interfaces belonging to this device.

        Format: list of `(interface_name, True)` tuples.
        Used by the Watchdog to track which interfaces are present.
        """
        subs = [(sub.topic_name, True) for sub in self.subscriber.topics]
        pubs = [(pub.topic_name, True) for pub in self.publisher.topics]
        acts = [(act._action_name, True) for act in self.actions.clients]
        servs = [(srv.service_name, True) for srv in self.services.clients]

        return subs + pubs + acts + servs
        
    # ===================================================================
    # SUBSCRIBER GETTERS
    # ===================================================================

    def get_position(self):
        """Return current robot pose (x, y in cm, angle in degrees)."""
        return self.subscriber.position.data

    def get_ir_proximity(self) -> list[int]:
        """Return the 7 IR proximity sensor readings."""
        return self.subscriber.ir_values.data

    def get_bumpers(self):
        """Return bumper hazard states."""
        return self.subscriber.bumpers

    def get_cliff_sensors(self):
        """Return cliff sensor states."""
        return self.subscriber.cliff_sensors

    def get_touch_sensors(self):
        """Return physical button states on the robot."""
        return self.subscriber.buttons

    def get_battery_level(self) -> float:
        """Return battery percentage (0–100)."""
        return self.subscriber.battery

    def get_accelerometer(self):
        """Return linear acceleration (x, y, z)."""
        return self.subscriber.acceleration.data

    def get_docking_values(self):
        """Return docking sensor information."""
        return self.subscriber.docking_values
    
    # ===================================================================
    # PUBLISHER COMMANDS
    # ===================================================================

    def set_lights_on_rgb(self, r: int, g: int, b: int) -> None:
        """Set all six LEDs to the same RGB color.

        Values for r, g, b must be in the range 0–255.
        """
        led = LedColor(red=r, green=g, blue=b)
        led_msg = LightringLeds()
        led_msg.override_system = True
        led_msg.leds = [led] * 6

        self.publisher.lightring = led_msg

    def set_lights(self, leds: list[LedColor]) -> None:
        """Set each of the six LEDs to a custom color.

        `leds` must be a list of exactly 6 `LedColor` objects.
        """
        led_msg = LightringLeds()
        led_msg.override_system = True
        led_msg.leds = leds

        self.publisher.lightring = led_msg

    def set_lights_off(self) -> None:
        """Turn off all Lightring LEDs."""
        self.publisher.lightring(LightringLeds())
        
    def set_wheel_speeds(self, left_wheel: float | int, right_wheel: float | int) -> None:
        """Set both wheel speeds in cm/s (range approximately -46 to +46 cm/s).

        This is the primary way to drive the robot. The background timer
        will continuously publish the command.
        """
        twist = Twist()
        # Convert cm/s to m/s and compute differential-drive kinematics
        twist.linear.x = ((right_wheel + left_wheel) / 100.0) / 2.0
        twist.angular.z = (right_wheel - left_wheel) / tools.constraints.WHEEL_DISTANCE_APART

        self.publisher.velocity = twist

    def set_left_speed(self, speed: float | int) -> None:
        """Set only the left wheel speed in cm/s (right wheel is kept from last command)."""
        if self.publisher.velocity == Twist():
            right_wheel = 0.0
        else:
            right_wheel = ((self.publisher.velocity.linear.x * 100.0) + (tools.constraints.WHEEL_DISTANCE_APART * self.publisher.velocity.angular.z) / 2.0)
        self.set_wheel_speeds(speed, right_wheel)

    def set_right_speed(self, speed: float | int) -> None:
        """Set only the right wheel speed in cm/s (left wheel is kept from last command)."""
        if self.publisher.velocity == Twist():
            left_wheel = 0.0
        else:
            left_wheel = ((self.publisher.velocity.linear.x * 100.0) - (tools.constraints.WHEEL_DISTANCE_APART * self.publisher.velocity.angular.z) / 2.0)
        self.set_wheel_speeds(left_wheel, speed)

    # ===================================================================
    # ACTION COMMANDS (High-level movement)
    # ===================================================================
    
    def set_lights_spin_rgb(self, r: int, g: int, b: int) -> None:
        """Start a spinning LED animation with the given RGB color.

        The animation runs for up to 500 seconds (or until cancelled).
        """
        led = LedColor(red=r, green=g, blue=b)
        led_msg = LedAnimation.Goal()
        led_msg.animation_type = LedAnimation.Goal.SPIN_LIGHTS
        led_msg.lightring.override_system = True
        led_msg.lightring.leds = [led, led, led, LedColor(), LedColor(), LedColor()]
        led_msg.max_runtime = Duration(sec=500, nanosec=0)

        self.actions.send_led_animation(led_msg)

    def set_lights_blink_rgb(self, r: int, g: int, b: int) -> None:
        """Start a blinking LED animation with the given RGB color.

        The animation runs for up to 500 seconds (or until cancelled).
        """
        led = LedColor(red=r, green=g, blue=b)
        led_msg = LedAnimation.Goal()
        led_msg.animation_type = LedAnimation.Goal.BLINK_LIGHTS
        led_msg.lightring.override_system = True
        led_msg.lightring.leds = [led] * 6
        led_msg.max_runtime = Duration(sec=500, nanosec=0)

        self.actions.send_led_animation(led_msg)
    
    def play_note(self, frequency: float | int, duration: float | int) -> None:
        """Play a single tone at the given frequency (Hz) for the given duration (seconds)."""
        sec = int(duration)
        nanosec = round((duration - sec) * 1_000_000_000)

        note = AudioNote(frequency=frequency, max_runtime=Duration(sec=sec, nanosec=nanosec))

        audio_msg = AudioNoteSequence.Goal()
        audio_msg.note_sequence.append = False
        audio_msg.note_sequence.notes = [note]

        self.actions.send_audio_note_sequence(audio_msg)
        
    def dock(self) -> None:
        """Request the robot to dock with the dock station."""
        self.actions.send_dock(Dock.Goal())

    def undock(self) -> None:
        """Request the robot to undock from the dock station."""
        self.actions.send_undock(Undock.Goal())
        
    def navigate_to(self, x: float | int, y: float | int, heading: float | int | None = None, speed: float | int = 20, use_goal: bool = True) -> None:
        """Navigate to a world coordinate (x, y) in centimeters.

        If `heading` is provided, the robot will face that direction (degrees)
        after arriving. Otherwise it will point along the line of travel.

        Units:
            x, y     → cm
            heading  → degrees (-180 to 180)
            speed    → cm/s
        """
        self.set_wheel_speeds(0, 0)  # stop any previous velocity commands

        direction = find_direction((x, y), self.get_position())

        # Heading correction (if requested)
        dif_w = math.radians(heading - direction.angle) if heading is not None else 0.0

        radius = tools.constraints.WHEEL_DISTANCE_APART / 2.0
        angular_speed = speed / radius

        if use_goal:
            nav_msg = NavigateToPosition.Goal()
            nav_msg.goal_pose.pose.position.x = x / 100.0
            nav_msg.goal_pose.pose.position.y = y / 100.0
            nav_msg.max_translation_speed = speed / 100.0
            nav_msg.max_rotation_speed = angular_speed

            if heading is not None:
                nav_msg.achieve_goal_heading = True
                orientation = convert_to_quaternion(0.0, 0.0, math.radians(heading))
                nav_msg.goal_pose.pose.orientation.z = orientation.z
                nav_msg.goal_pose.pose.orientation.w = orientation.w
            else:
                nav_msg.achieve_goal_heading = False

            self.actions.send_navigate_to_position(nav_msg)

            # Estimated time for the whole maneuver
            t = (abs(direction.angle / angular_speed) + abs(direction.distance / speed) + abs(dif_w / angular_speed))
            time.sleep(DEFAULT_WAIT + t)

        else:
            # Manual fallback (turn → drive → final heading)
            turn_dir = direction.angle / abs(direction.angle) if direction.angle != 0 else 0
            if turn_dir == -1:
                self.turn_right(math.degrees(direction.angle), speed, use_goal=False)
            elif turn_dir == 1:
                self.turn_left(math.degrees(direction.angle), speed, use_goal=False)

            if direction.distance > 0:
                self.move(direction.distance, speed, use_goal=False)

            if heading is not None:
                turn_dir = dif_w / abs(dif_w)
                if turn_dir == -1:
                    self.turn_right(math.degrees(dif_w), speed, use_goal=False)
                elif turn_dir == 1:
                    self.turn_left(math.degrees(dif_w), speed, use_goal=False)
        
    def turn_left(self, angle: float | int, speed: float | int = 20, use_goal: bool = True) -> None:
        """Rotate left by `angle` degrees at the given speed (cm/s)."""
        self.set_wheel_speeds(0, 0)

        radius = tools.constraints.WHEEL_DISTANCE_APART / 2.0
        twist_msg = tools.velocity.get_twist(abs(speed), radius)
        t = tools.velocity.get_motion_time(twist_msg, angle_deg=abs(angle))

        if use_goal:
            rotate_msg = RotateAngle.Goal()
            rotate_msg.angle = abs(math.radians(angle))
            rotate_msg.max_rotation_speed = twist_msg.angular.z

            self.actions.send_rotate_angle(rotate_msg)
            time.sleep(DEFAULT_WAIT + t)
        else:
            self._run_twist(twist_msg, t)

    def turn_right(self, angle: float | int, speed: float | int = 20, use_goal: bool = True) -> None:
        """Rotate right by `angle` degrees at the given speed (cm/s)."""
        self.set_wheel_speeds(0, 0)

        radius = tools.constraints.WHEEL_DISTANCE_APART / 2.0
        twist_msg = tools.velocity.get_twist(-abs(speed), radius)
        t = tools.velocity.get_motion_time(twist_msg, angle_deg=abs(angle))

        if use_goal:
            rotate_msg = RotateAngle.Goal()
            rotate_msg.angle = -abs(math.radians(angle))
            rotate_msg.max_rotation_speed = twist_msg.angular.z

            self.actions.send_rotate_angle(rotate_msg)
            time.sleep(DEFAULT_WAIT + t)
        else:
            self._run_twist(twist_msg, t)
            
    def move(self, distance: float | int, speed: float | int = 20, use_goal: bool = True) -> None:
        """Drive straight for `distance` cm at the given speed (cm/s)."""
        self.set_wheel_speeds(0, 0)

        twist_msg = tools.velocity.get_twist(speed, 0.0)
        t = tools.velocity.get_motion_time(twist_msg, distance_cm=abs(distance))

        if use_goal:
            move_msg = DriveDistance.Goal()
            move_msg.distance = abs(distance) / 100.0
            move_msg.max_translation_speed = speed / 100.0

            self.actions.send_drive_distance(move_msg)
            time.sleep(DEFAULT_WAIT + t)
        else:
            self._run_twist(twist_msg, t)
            
    def arc_left(self, angle: float | int, radius: float | int, direction: int = 1, speed: float | int = 20, use_goal: bool = True) -> None:
        """Drive a left arc of `angle` degrees with given radius (cm).

        `direction` = 1 (forward) or -1 (backward).
        """
        self.set_wheel_speeds(0, 0)

        if direction not in (1, -1):
            raise ValueError("Direction must be 1 (forward) or -1 (backward)")

        twist_msg = tools.velocity.get_twist(abs(speed) * direction, abs(radius))
        t = tools.velocity.get_motion_time(twist_msg, angle_deg=abs(angle))

        if use_goal:
            arc_msg = DriveArc.Goal()
            arc_msg.angle = abs(math.radians(angle))
            arc_msg.radius = abs(radius) / 100.0
            arc_msg.max_translation_speed = abs(speed)
            arc_msg.translate_direction = direction

            self.actions.send_drive_arc(arc_msg)
            time.sleep(DEFAULT_WAIT + t)
        else:
            self._run_twist(twist_msg, t)

    def arc_right(self, angle: float | int, radius: float | int, direction: int = 1, speed: float | int = 20, use_goal: bool = True) -> None:
        """Drive a right arc of `angle` degrees with given radius (cm).

        `direction` = 1 (forward) or -1 (backward).
        """
        self.set_wheel_speeds(0, 0)

        if direction not in (1, -1):
            raise ValueError("Direction must be 1 (forward) or -1 (backward)")

        twist_msg = tools.velocity.get_twist(abs(speed) * direction, -abs(radius))
        t = tools.velocity.get_motion_time(twist_msg, angle_deg=abs(angle))

        if use_goal:
            arc_msg = DriveArc.Goal()
            arc_msg.angle = -abs(math.radians(angle))
            arc_msg.radius = abs(radius) / 100.0
            arc_msg.max_translation_speed = abs(speed)
            arc_msg.translate_direction = direction

            self.actions.send_drive_arc(arc_msg)
            time.sleep(DEFAULT_WAIT + t)
        else:
            self._run_twist(twist_msg, t)
            
    def _run_twist(self, twist_msg: Twist, wait_time: float) -> None:
        """Publish a twist command repeatedly for `wait_time` seconds.

        Used when `use_goal=False` for manual timed movement.
        """
        loop = math.floor(wait_time / 0.2)
        remainder = wait_time - (loop * 0.2)

        for _ in range(loop):
            self.publisher.send_velocity(twist_msg)
            time.sleep(0.199)

        self.publisher.send_velocity(twist_msg)
        time.sleep(remainder)
        self.publisher.send_velocity(Twist())  # stop

        time.sleep(0.5)  # final safety buffer

    # ===================================================================
    # SERVICE COMMANDS
    # ===================================================================

    def reset_navigation(self) -> None:
        """Request the robot to reset its position and heading to (0, 0, 0°).

        This is typically called once at startup. The robot takes up to ~4 seconds
        to complete the reset.
        """
        # self.print_warning("Resetting robot position. Max time 4 sec.")

        # Send the reset request
        self.services.send_reset_navigation(ResetPose.Request())

        # Give the robot time to process the reset
        time.sleep(1.0)