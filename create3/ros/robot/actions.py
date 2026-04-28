#
# Action Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time, math
from typing import TYPE_CHECKING

from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient as CreateActionClient
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from irobot_create_msgs.msg import AudioNoteVector, LedColor, AudioNote
from irobot_create_msgs.action import NavigateToPosition, DriveArc, DriveDistance, RotateAngle, Dock, Undock, LedAnimation, AudioNoteSequence

from create3.models.robot import Subscribe
import create3.utils.robot as tools
from create3.utils.common.coords import convert_to_quaternion, find_direction
from create3.utils import Threading
from create3.utils.common.other import TIMEOUT, DEFAULT_WAIT

from .callbacks.goal import (
    goal_response_callback,
)

class ActionClient(Threading if TYPE_CHECKING else object):
    """ROS action client manager for the iRobot Create3.

    Provides high-level control over:
      • LED animations (spin, blink)
      • Audio sequences
      • Navigation, driving, turning, docking/undocking

    All action clients use a mutually exclusive callback group so they never
    interfere with subscriptions or other callbacks. The class also registers
    itself with the debugger for interface monitoring.
    """

    _velocities: Publisher
    _subscription_msgs: Subscribe

    def __init__(self, node: Node) -> None:
        """Initialize all action clients and wait for the servers to become available.

        Parameters
        ----------
        node : Node
            The ROS node that owns these action clients.
        """
        super().__init__(node)  # initialize Threading + Logger

        self._use_goal = True

        # Use a mutually exclusive callback group so action calls never block
        # other callbacks (subscriptions, timers, etc.)
        action_callback_group = MutuallyExclusiveCallbackGroup()

        # Create action clients
        self._led_animate = CreateActionClient(self.node, LedAnimation, "led_animation", callback_group=action_callback_group)
        self._audio_sequence = CreateActionClient(self.node, AudioNoteSequence, "audio_note_sequence", callback_group=action_callback_group)
        self._navigate = CreateActionClient(self.node, NavigateToPosition, "navigate_to_position", callback_group=action_callback_group)
        self._drive_arc = CreateActionClient(self.node, DriveArc, "drive_arc", callback_group=action_callback_group)
        self._drive_distance = CreateActionClient(self.node, DriveDistance, "drive_distance", callback_group=action_callback_group)
        self._rotate_angle = CreateActionClient(self.node, RotateAngle, "rotate_angle", callback_group=action_callback_group)
        self._dock = CreateActionClient(self.node, Dock, "dock", callback_group=action_callback_group)
        self._undock = CreateActionClient(self.node, Undock, "undock", callback_group=action_callback_group)

        # Wait for all action servers
        self._led_animate.wait_for_server(timeout_sec=TIMEOUT)
        self._audio_sequence.wait_for_server(timeout_sec=TIMEOUT)
        self._navigate.wait_for_server(timeout_sec=TIMEOUT)
        self._drive_arc.wait_for_server(timeout_sec=TIMEOUT)
        self._drive_distance.wait_for_server(timeout_sec=TIMEOUT)
        self._rotate_angle.wait_for_server(timeout_sec=TIMEOUT)
        self._dock.wait_for_server(timeout_sec=TIMEOUT)
        self._undock.wait_for_server(timeout_sec=TIMEOUT)

        # Register with debugger for interface monitoring
        self.debug.actions = [
            self._led_animate,
            self._audio_sequence,
            self._navigate,
            self._drive_arc,
            self._drive_distance,
            self._rotate_angle,
            self._dock,
            self._undock,
        ]

    # =====================================================================
    # LED Animations
    # =====================================================================

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

        future = self._led_animate.send_goal_async(led_msg)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))

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

        future = self._led_animate.send_goal_async(led_msg)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))

    # =====================================================================
    # Audio
    # =====================================================================

    def play_note(self, frequency: float | int, duration: float | int) -> None:
        """Play a single tone at the given frequency (Hz) for the given duration (seconds)."""
        sec = int(duration)
        nanosec = round((duration - sec) * 1_000_000_000)

        note = AudioNote(frequency=frequency, max_runtime=Duration(sec=sec, nanosec=nanosec))

        audio_msg = AudioNoteSequence.Goal()
        audio_msg.note_sequence.append = False
        audio_msg.note_sequence.notes = [note]

        future = self._audio_sequence.send_goal_async(audio_msg)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))
        
    # def play_note_sequence(self):
    #     """PLay note with frequency in hertz for duration in seconds."""

    #     audio_msg = AudioNoteSequence.Goal()
    #     audio_msg.note_sequence = AudioNoteVector()
    #     audio_msg.iterations = 1
    #     audio_msg.note_sequence.notes = []
        
    #     music = MarioTheme()
        
    #     for note in music.notes:
    #         duration = Duration(sec=int(note.length), nanosec=round((note.length - int(note.length)) * 1000000000))
            
    #         duration_spacer = Duration(sec=int(note.delay), nanosec=round((note.delay - int(note.delay)) * 1000000000))
            
    #         audio_msg.note_sequence.notes += [AudioNote(frequency=note.frequency, max_runtime=duration)]
    #         audio_msg.note_sequence.notes += [AudioNote(frequency=Note.REST, max_runtime=duration_spacer)]
            
    #     self._audio_sequence.send_goal(audio_msg)

    # =====================================================================
    # Docking
    # =====================================================================

    def dock(self) -> None:
        """Request the robot to dock with the dock station."""
        self._dock.send_goal(Dock.Goal())

    def undock(self) -> None:
        """Request the robot to undock from the dock station."""
        self._undock.send_goal(Undock.Goal())

    # =====================================================================
    # High-level navigation & movement
    # =====================================================================

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

        direction = find_direction((x, y), self._subscription_msgs.position.data)

        # Heading correction (if requested)
        dif_w = math.radians(heading - direction.angle) if heading is not None else 0.0

        radius = tools.constraints.WHEEL_DISTANCE_APART / 2.0
        angular_speed = speed / radius

        if use_goal and self._use_goal:
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

            future = self._navigate.send_goal_async(nav_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))

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

    # =====================================================================
    # Primitive movement commands
    # =====================================================================

    def turn_left(self, angle: float | int, speed: float | int = 20, use_goal: bool = True) -> None:
        """Rotate left by `angle` degrees at the given speed (cm/s)."""
        self.set_wheel_speeds(0, 0)

        radius = tools.constraints.WHEEL_DISTANCE_APART / 2.0
        twist_msg = tools.velocity.get_twist(abs(speed), radius)
        t = tools.velocity.get_motion_time(twist_msg, angle_deg=abs(angle))

        if use_goal and self._use_goal:
            rotate_msg = RotateAngle.Goal()
            rotate_msg.angle = abs(math.radians(angle))
            rotate_msg.max_rotation_speed = twist_msg.angular.z

            future = self._rotate_angle.send_goal_async(rotate_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            time.sleep(DEFAULT_WAIT + t)
        else:
            self._run_twist(twist_msg, t)

    def turn_right(self, angle: float | int, speed: float | int = 20, use_goal: bool = True) -> None:
        """Rotate right by `angle` degrees at the given speed (cm/s)."""
        self.set_wheel_speeds(0, 0)

        radius = tools.constraints.WHEEL_DISTANCE_APART / 2.0
        twist_msg = tools.velocity.get_twist(-abs(speed), radius)
        t = tools.velocity.get_motion_time(twist_msg, angle_deg=abs(angle))

        if use_goal and self._use_goal:
            rotate_msg = RotateAngle.Goal()
            rotate_msg.angle = -abs(math.radians(angle))
            rotate_msg.max_rotation_speed = twist_msg.angular.z

            future = self._rotate_angle.send_goal_async(rotate_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            time.sleep(DEFAULT_WAIT + t)
        else:
            self._run_twist(twist_msg, t)

    def move(self, distance: float | int, speed: float | int = 20, use_goal: bool = True) -> None:
        """Drive straight for `distance` cm at the given speed (cm/s)."""
        self.set_wheel_speeds(0, 0)

        twist_msg = tools.velocity.get_twist(speed, 0.0)
        t = tools.velocity.get_motion_time(twist_msg, distance_cm=abs(distance))

        if use_goal and self._use_goal:
            move_msg = DriveDistance.Goal()
            move_msg.distance = abs(distance) / 100.0
            move_msg.max_translation_speed = speed / 100.0

            future = self._drive_distance.send_goal_async(move_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
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

        if use_goal and self._use_goal:
            arc_msg = DriveArc.Goal()
            arc_msg.angle = abs(math.radians(angle))
            arc_msg.radius = abs(radius) / 100.0
            arc_msg.max_translation_speed = abs(speed)
            arc_msg.translate_direction = direction

            future = self._drive_arc.send_goal_async(arc_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
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

        if use_goal and self._use_goal:
            arc_msg = DriveArc.Goal()
            arc_msg.angle = -abs(math.radians(angle))
            arc_msg.radius = abs(radius) / 100.0
            arc_msg.max_translation_speed = abs(speed)
            arc_msg.translate_direction = direction

            future = self._drive_arc.send_goal_async(arc_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            time.sleep(DEFAULT_WAIT + t)
        else:
            self._run_twist(twist_msg, t)

    # =====================================================================
    # Internal helper
    # =====================================================================

    def _run_twist(self, twist_msg: Twist, wait_time: float) -> None:
        """Publish a twist command repeatedly for `wait_time` seconds.

        Used when `use_goal=False` for manual timed movement.
        """
        loop = math.floor(wait_time / 0.5)
        remainder = wait_time - (loop * 0.5)

        for _ in range(loop):
            self._velocities.publish(twist_msg)
            time.sleep(0.499)

        self._velocities.publish(twist_msg)
        time.sleep(remainder)
        self._velocities.publish(Twist())  # stop

        time.sleep(0.5)  # final safety buffer