#
# Action Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from rclpy.action import ActionClient as CreateActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from irobot_create_msgs.action import NavigateToPosition, DriveArc, DriveDistance, RotateAngle, Dock, Undock, LedAnimation, AudioNoteSequence

from create3.models.robot import Actions
from create3.utils import Logger, Node
from create3.utils.common.other import TIMEOUT

from .callbacks import (
    goal_response_callback,
)

class ActionClient(Logger):
    """ROS action client manager for the iRobot Create3.

    Provides high-level control over:
      • LED animations (spin, blink)
      • Audio sequences
      • Navigation, driving, turning, docking/undocking

    All action clients use a mutually exclusive callback group so they never
    interfere with subscriptions or other callbacks. The class also registers
    itself with the debugger for interface monitoring.
    """

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
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Register with debugger for interface monitoring
        self.clients: list[CreateActionClient] = []
        
    def find(self, name: Actions) -> CreateActionClient:
        for action in self.clients:
            if name == action._action_name:
                return action
            
        return None
    
    # =====================================================================
    # LED Animations
    # =====================================================================
    
    def send_led_animation(self, led_animation_goal: LedAnimation.Goal):
        if not self.find(Actions.LED_ANIMATION):
            client = CreateActionClient(self.node, LedAnimation, Actions.LED_ANIMATION, callback_group=self.callback_group)
            client.wait_for_server(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        future = self.find(Actions.LED_ANIMATION).send_goal_async(led_animation_goal)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))

    # =====================================================================
    # Audio
    # =====================================================================
    
    def send_audio_note_sequence(self, audio_note_sequence_goal: AudioNoteSequence.Goal):
        if not self.find(Actions.AUDIO_NOTE_SEQUENCE):
            client = CreateActionClient(self.node, AudioNoteSequence, Actions.AUDIO_NOTE_SEQUENCE, callback_group=self.callback_group)
            client.wait_for_server(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        future = self.find(Actions.AUDIO_NOTE_SEQUENCE).send_goal_async(audio_note_sequence_goal)
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
    
    def send_dock(self, dock_goal: Dock.Goal):
        if not self.find(Actions.DOCK):
            client = CreateActionClient(self.node, Dock, Actions.DOCK, callback_group=self.callback_group)
            client.wait_for_server(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        self.find(Actions.DOCK).send_goal(dock_goal)
        
    def send_undock(self, undock_goal: Undock.Goal):
        if not self.find(Actions.UNDOCK):
            client = CreateActionClient(self.node, Undock, Actions.UNDOCK, callback_group=self.callback_group)
            client.wait_for_server(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        self.find(Actions.UNDOCK).send_goal(undock_goal)

    # =====================================================================
    # High-level navigation & movement
    # =====================================================================
    
    def send_navigate_to_position(self, navigate_to_position_goal: NavigateToPosition.Goal):
        if not self.find(Actions.NAVIGATE_TO_POSITION):
            client = CreateActionClient(self.node, NavigateToPosition, Actions.NAVIGATE_TO_POSITION, callback_group=self.callback_group)
            client.wait_for_server(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        future = self.find(Actions.NAVIGATE_TO_POSITION).send_goal_async(navigate_to_position_goal)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))

    # =====================================================================
    # Primitive movement commands
    # =====================================================================
    
    def send_rotate_angle(self, rotate_angle_goal: RotateAngle.Goal):
        if not self.find(Actions.ROTATE_ANGLE):
            client = CreateActionClient(self.node, RotateAngle, Actions.ROTATE_ANGLE, callback_group=self.callback_group)
            client.wait_for_server(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        future = self.find(Actions.ROTATE_ANGLE).send_goal_async(rotate_angle_goal)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))
        
    def send_drive_distance(self, drive_distance_goal: DriveDistance.Goal):
        if not self.find(Actions.DRIVE_DISTANCE):
            client = CreateActionClient(self.node, DriveDistance, Actions.DRIVE_DISTANCE, callback_group=self.callback_group)
            client.wait_for_server(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        future = self.find(Actions.DRIVE_DISTANCE).send_goal_async(drive_distance_goal)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))
        
    def send_drive_arc(self, drive_arc_goal: DriveArc.Goal):
        if not self.find(Actions.DRIVE_ARC):
            client = CreateActionClient(self.node, DriveArc, Actions.DRIVE_ARC, callback_group=self.callback_group)
            client.wait_for_server(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        future = self.find(Actions.DRIVE_ARC).send_goal_async(drive_arc_goal)
        future.add_done_callback(lambda msg: goal_response_callback(self, msg))
