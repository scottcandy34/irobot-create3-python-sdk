#
# Action Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time, math
from typing import TYPE_CHECKING

from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient as CreateActionClient
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from irobot_create_msgs.msg import AudioNoteVector, LedColor, AudioNote
from irobot_create_msgs.action import NavigateToPosition, DriveArc, DriveDistance, RotateAngle, Dock, Undock, LedAnimation, AudioNoteSequence

from create3.models.robot import Subscribe
import create3.utils.robot as tools
from create3.utils.common import convert_to_quaternion
from create3.utils import Threading, TIMEOUT, DEFAULT_WAIT

from .callbacks.goal import (
    goal_response_callback,
)

class ActionClient(Threading if TYPE_CHECKING else object):
    """Setup ROS action clients, and handle goals."""

    _velocities: Publisher
    _subscription_msgs: Subscribe

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten
        self._use_goal = True

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        action_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Action Clients
        self._led_animate = CreateActionClient(self.node, LedAnimation, 'led_animation', callback_group=action_callback_group)
        self._led_animate.wait_for_server(TIMEOUT)
        self._audio_sequence = CreateActionClient(self.node, AudioNoteSequence, 'audio_note_sequence', callback_group=action_callback_group)
        self._audio_sequence.wait_for_server(TIMEOUT)
        self._navigate = CreateActionClient(self.node, NavigateToPosition, 'navigate_to_position', callback_group=action_callback_group)
        self._navigate.wait_for_server(TIMEOUT)
        self._drive_arc = CreateActionClient(self.node, DriveArc, 'drive_arc', callback_group=action_callback_group)
        self._drive_arc.wait_for_server(TIMEOUT)
        self._drive_distance = CreateActionClient(self.node, DriveDistance, 'drive_distance', callback_group=action_callback_group)
        self._drive_distance.wait_for_server(TIMEOUT)
        self._rotate_angle = CreateActionClient(self.node, RotateAngle, 'rotate_angle', callback_group=action_callback_group)
        self._rotate_angle.wait_for_server(TIMEOUT)
        self._dock = CreateActionClient(self.node, Dock, 'dock', callback_group=action_callback_group)
        self._dock.wait_for_server(TIMEOUT)
        self._undock = CreateActionClient(self.node, Undock, 'undock', callback_group=action_callback_group)
        self._undock.wait_for_server(TIMEOUT)

        # Add actions to debugger
        self.debug.actions = [self._led_animate, self._audio_sequence, self._navigate, self._drive_arc, self._drive_distance, self._rotate_angle, self._dock, self._undock]

    def set_lights_spin_rgb(self, r: int, g: int, b: int):
        """Set robot's LED to spin with desired color red, green, blue."""
        
        # Set individual LED color
        led1 = LedColor(red=r, green=g, blue=b)
        led2 = LedColor(red=r, green=g, blue=b)
        led3 = LedColor(red=r, green=g, blue=b)
        led4 = LedColor()
        led5 = LedColor()
        led6 = LedColor()
        
        # Create animation goal message
        led_msg = LedAnimation.Goal()
        led_msg.animation_type = LedAnimation.Goal.SPIN_LIGHTS
        led_msg.lightring.override_system = True
        led_msg.lightring.leds = [led1, led2, led3, led4, led5, led6]
        led_msg.max_runtime = Duration(sec = 500, nanosec = 0) # how long animation lasts
        
        future = self._led_animate.send_goal_async(led_msg)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))
        
    def set_lights_blink_rgb(self, r: int, g: int, b: int):
        """Set robot's LED to blink with desired color red, gree, blue."""
        
        # Set individual LED color
        led1 = LedColor(red=r, green=g, blue=b)
        led2 = LedColor(red=r, green=g, blue=b)
        led3 = LedColor(red=r, green=g, blue=b)
        led4 = LedColor(red=r, green=g, blue=b)
        led5 = LedColor(red=r, green=g, blue=b)
        led6 = LedColor(red=r, green=g, blue=b)
        
        # Create animation goal message
        led_msg = LedAnimation.Goal()
        led_msg.animation_type = LedAnimation.Goal.BLINK_LIGHTS
        led_msg.lightring.override_system = True
        led_msg.lightring.leds = [led1, led2, led3, led4, led5, led6]
        # led_msg.max_runtime.sec = 500 # how long animation lasts
        led_msg.max_runtime = Duration(sec = 500, nanosec = 0)
        
        future = self._led_animate.send_goal_async(led_msg)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))
        
    def play_note(self, frequency: float | int, duration: float | int):
        """PLay note with frequency in hertz for duration in seconds."""
        
        # Set individual Note
        sec = int(duration)
        nanosec = round((duration - sec) * 1000000000)
        note = AudioNote(frequency=frequency, max_runtime=Duration(sec=sec, nanosec=nanosec))
        
        # Create audio message
        audio_msg = AudioNoteVector()
        audio_msg.append = False
        audio_msg.notes = [note]
        
        future = self._audio_sequence.send_goal_async(audio_msg)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))
        
    def dock(self):
        """Request a docking action"""
        
        # Create Dock Goal message
        dockingMsg = Dock.Goal()
        
        # Send Dock Goal message
        self._dock.send_goal(dockingMsg)
        
    def undock(self):
        """Request a undocking action"""
        
        # Create Undock Goal message
        undockingMsg = Undock.Goal()
        
        # Send Undock Goal message
        self._undock.send_goal(undockingMsg)
        
    def navigate_to(self, x: float | int, y: float | int, heading: float | int = None, speed: float | int = 20, use_goal: bool = True):
        """ If heading is None, then it will be ignored, and the robot will arrive to its destination
        pointing towards the direction of the line between the destination and the origin points.
        
        heading is -180 to 180
        
        Units:
            x, y: cm
            heading: deg
            speed: cm/s
            use_goal: bool
        """
        
        self.set_wheel_speeds(0,0) # Clear set wheel speeds
        
        # Calculate differences between current position and new position
        dif_x = (x - self._subscription_msgs.position.x) / 100 # Difference in X coords and convert to (m) | x(cm) - current_x(cm) / 100
        dif_y = (y - self._subscription_msgs.position.y) / 100 # Difference in Y coords and convert to (m) | y(cm) - current_y(cm) / 100
        dist = math.sqrt(dif_x**2 + dif_y**2) # Get distance to move to new coords (Pythagorean Theorem) | sqrt( difference_x(m)^2 + difference_y(m)^2 ) = distance(m)
        
        angle = math.atan2(dif_y, dif_x) - math.radians(self._subscription_msgs.position.angle) # Get angle (in radians) to Turn to for new coords | atan2( difference_y(m), difference_x(m) ) - current_angle(rad) = angle_facing_move(rad)
        dif_w = math.radians(heading - angle) if heading else 0 # Difference in W orientation (Heading) | heading(rad) - angle_facing_move(rad) = heading_angle_left(rad)
        
        radius = tools.constraints.WHEEL_DISTANCE_APART / 100 / 2 # convert to meters and divide by 2 to get radius
        speed = speed / 100 # convert to meters
        angularSpeed = speed / radius # speed(m/s) / radius(m) = angular_velocity(rad/s)
        
        if use_goal and self._use_goal:
            # Create Navigation Goal message
            nav_msg = NavigateToPosition.Goal()
            nav_msg.goal_pose.pose.position.x = x / 100 # Convert to meters
            nav_msg.goal_pose.pose.position.y = y / 100 # Convert to meters
            nav_msg.max_translation_speed = speed
            nav_msg.max_rotation_speed = angularSpeed
            
            # Configure Heading part of message
            if heading is not None:
                nav_msg.achieve_goal_heading = True # Tell robot to Turn to heading value
                orientation = convert_to_quaternion(0, 0, math.radians(heading)) # Convert heading to radians and convert from Euler angles to Quaternion rotations
                nav_msg.goal_pose.pose.orientation.z = orientation[2] # Set z value from Quaternion rotation
                nav_msg.goal_pose.pose.orientation.w = orientation[3] # Set w value from Quaternion rotation
            else:
                nav_msg.achieve_goal_heading = False # Tell robot to ignore heading value
            
            # Send Navigate Goal message
            future = self._navigate.send_goal_async(nav_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            
            # Calculate Time to complete goal
            # Turn and face the new coordinate point  |  Move distance to new coordinate point  |  Turn to new heading
            # angle(rad) / angular_velocity(rad/s)    +  distance(m) / linear_velocity(m/s)     +  angle(rad) / angular_velocity(rad/s)
            t = abs(angle / angularSpeed) + abs(dist / speed) + abs(dif_w / angularSpeed)
            time.sleep(DEFAULT_WAIT + t) # wait for goal to complete
        
        else:
            # First action turn to face new coords
            direction = angle / abs(angle)
            if direction == -1:
                self.turn_right(math.degrees(angle), speed * 100, use_goal=False)
            elif direction == 1:
                self.turn_left(math.degrees(angle), speed * 100, use_goal=False)
            
            # Second action move to new coords
            if dist > 0:
                self.move(dist * 100, speed * 100, use_goal=False)
                
            # Third and final action turn to new heading from current heading.        
            direction = dif_w / abs(dif_w) if heading else 0
            if heading and direction == -1:
                self.turn_right(math.degrees(angle), speed * 100, use_goal=False)
            elif heading and direction == 1:
                self.turn_left(math.degrees(angle), speed * 100, use_goal=False)
        
    def turn_left(self, angle: float | int, speed: float | int = 20, use_goal: bool = True):
        """Rotate left for specific angle in degrees. Speed in cm/s. use_goal to enable or disable goals."""
        
        self.set_wheel_speeds(0,0) # Clear set wheel speeds
        
        angle = abs(math.radians(angle)) # must be a positive to turn left and convert to radians
        radius = tools.constraints.WHEEL_DISTANCE_APART / 100 / 2 # convert to meters and divide by 2 to get radius
        angularSpeed = (speed / 100) / radius # speed(m/s) / radius(m) = angular_velocity(rad/s)
        # Calculate time
        t = abs(angle / angularSpeed) # time = angle(rad) / angular_velocity(rad/s)
        
        if use_goal and self._use_goal:
            # Create Rotate Goal message
            rotate_msg = RotateAngle.Goal()
            rotate_msg.angle = angle
            rotate_msg.max_rotation_speed = angularSpeed
            
            # Send Rotate Goal message
            future = self._rotate_angle.send_goal_async(rotate_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            
            # Calculate time and wait
            time.sleep(DEFAULT_WAIT + t)
        
        else:
            # Create Twist message
            twist_msg = Twist()
            twist_msg.angular.z = angularSpeed * angle / abs(angle)
            
            # Send twist with time
            self._run_twist(twist_msg, t)
        
    def turn_right(self, angle: float | int, speed: float | int = 20, use_goal: bool = True):
        """Rotate right for specific angle in degrees. Speed in cm/s. use_goal to enable or disable goals."""
        
        self.set_wheel_speeds(0,0) # Clear set wheel speeds
        
        angle = -abs(math.radians(angle)) # must be a negative to turn right and convert to radians
        radius = tools.constraints.WHEEL_DISTANCE_APART / 100 / 2 # convert to meters and divide by 2 to get radius
        angularSpeed = (speed / 100) / radius # speed(m/s) / radius(m) = angular_velocity(rad/s)
        # Calculate time
        t = abs(angle / angularSpeed) # time = angle(rad) / angular_velocity(rad/s)
        
        if use_goal and self._use_goal:
            # Create Rotate Goal message
            rotate_msg = RotateAngle.Goal()
            rotate_msg.angle = angle
            rotate_msg.max_rotation_speed = angularSpeed
            
            # Send Rotate Goal message and wait
            future = self._rotate_angle.send_goal_async(rotate_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            
            # Calculate time and wait
            time.sleep(DEFAULT_WAIT + t)
        
        else:
            # Create Twist message
            twist_msg = Twist()
            twist_msg.angular.z = angularSpeed * angle / abs(angle)
            
            # Send twist with time
            self._run_twist(twist_msg, t)
    
    def move(self, distance: float | int, speed: float | int = 20, use_goal: bool = True):
        """Drive distance in centimeters. Speed in cm/s. use_goal to enable or disable goals."""
        
        self.set_wheel_speeds(0,0) # Clear set wheel speeds
        
        distance = distance / 100 # convert to meters
        speed = speed / 100 # convert to meters
        # Calculate time
        t = abs(distance / speed) # time = distance(m) / linear_velocity(m/s)
        
        if use_goal and self._use_goal:
            # Create Drive Distance Goal message
            move_msg = DriveDistance.Goal()
            move_msg.distance = distance
            move_msg.max_translation_speed = speed
            
            # Send Drive Distance Goal message and wait
            future = self._drive_distance.send_goal_async(move_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            
            # Calculate time and wait
            time.sleep(DEFAULT_WAIT + t)
        else:
            # Create Twist message
            twist_msg = Twist()
            twist_msg.linear.x = speed # Just set linear_velocity(m/s)
            
            # Send twist with time
            self._run_twist(twist_msg, t)

    def arc_left(self, angle: float | int, radius: float | int, direction: int = 1, speed: float | int = 20, use_goal: bool = True):
        """Drive arc 'left' defined by angle in degrees and radius in cm.
        
        direction is 1 for forward and -1 for backwards
        
        Speed in cm/s. use_goal to enable or disable goals.
        """
        
        self.set_wheel_speeds(0,0) # Clear set wheel speeds
        
        # Checks if direction input is valid
        if direction != 1 and direction != -1:
            raise Exception("Direction must be 1 or -1")
        
        angle = abs(math.radians(angle)) * direction # must be a positive to turn left and convert to radians
        radius = abs(radius / 100) # convert to meters and must be positive
        speed = speed / 100 # convert to meters
        # Calculate time
        t = abs(angle * radius / speed) # time = angle(rad) * radius(m) / linear_velocity(m/s)
        
        
        if use_goal and self._use_goal:
            # Create Drive Arc Goal message
            arc_msg = DriveArc.Goal()
            arc_msg.angle = angle # must be a positive to turn left and convert to radians
            arc_msg.radius = radius # convert to meters and must be positive
            arc_msg.max_translation_speed = speed
            arc_msg.translate_direction = direction # sets direction to move
            
            # Send Drive Arc Goal message and wait
            future = self._drive_arc.send_goal_async(arc_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            
            # Wait for goal to finish with calculated time
            time.sleep(DEFAULT_WAIT + t)
        
        else:
            # Create Twist message
            twist_msg = Twist()
            twist_msg.angular.z = speed / radius # speed(m/s) / radius(m) = angular_velocity(rad/s)
            twist_msg.linear.x = speed * direction
            
            # Send twist with time
            self._run_twist(twist_msg, t)
        
    def arc_right(self, angle: float | int, radius: float | int, direction: int = 1, speed: float | int = 20, use_goal: bool = True):
        """Drive arc 'right' defined by angle in degrees and radius in cm.
        
        direction is 1 for forward and -1 for backwards
        
        Speed in cm/s. use_goal to enable or disable goals.
        """
        
        self.set_wheel_speeds(0,0) # Clear set wheel speeds
        
        if direction != 1 and direction != -1:
            raise Exception("Direction must be 1 or -1")
        
        angle = -abs(math.radians(angle)) * direction # must be a negative to turn right and convert to radians
        radius = abs(radius / 100) # convert to meters and must be negative
        speed = speed / 100 # convert to meters
        # Calculate time
        t = abs(angle * radius / speed) # time = angle(rad) * radius(m) / linear_velocity(m/s)
        
        if use_goal and self._use_goal:
            # Create Drive Arc Goal message
            arc_msg = DriveArc.Goal()
            arc_msg.angle = angle
            arc_msg.radius = radius
            arc_msg.max_translation_speed = speed
            arc_msg.translate_direction = direction # sets direction to move
            
            # Send Drive Arc Goal message and wait
            future = self._drive_arc.send_goal_async(arc_msg)
            future.add_done_callback(lambda msg: goal_response_callback(self, msg))
            
            # Wait for goal to finish with calculated time
            time.sleep(DEFAULT_WAIT + t)
        
        else:
            # Create Twist message
            twist_msg = Twist()
            twist_msg.angular.z = -speed / radius # speed(m/s) / radius(m) = angular_velocity(rad/s)
            twist_msg.linear.x = speed * direction
            
            # Send twist with time
            self._run_twist(twist_msg, t)
        
    def _run_twist(self, twist_msg: Twist, waitTime: float):
        """Runs twist until time is completed. Time is in seconds."""
        loop = math.floor(waitTime / 0.5) # Divides waitTime by 0.5s since thats how long a twist command runs for. Returns only whole numbers rounded down
        remainder = waitTime - (loop * 0.5) # Finds the remainder waitTime that was less than 0.5s
        
        # Publish twist every 0.5s
        for i in range(loop):
            self._velocities.publish(twist_msg)
            time.sleep(0.499) # This is just less than 0.5s so it can account for the time it takes in the loop and then publish again.
        
        # Send final twist message
        self._velocities.publish(twist_msg)
        time.sleep(remainder) # Waits for the remainder of the time 
        self._velocities.publish(Twist()) # Sends empty twist to stop the robot at exactly the correct timing.
        
        # Final wait to not get interrupted by possible other commands
        time.sleep(0.5)
