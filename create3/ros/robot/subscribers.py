#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, InterfaceButtons, DockStatus, IrOpcode

from .callbacks import MessageHandler
from create3.utils import Threading
from create3.models.common import Position
from create3.models.robot import HazardBumper, HazardCliff, Acceleration, DockingValues

qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(MessageHandler, Threading if TYPE_CHECKING else object):
    """Handles ROS subscribers for robot data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Subscription
        self._odom = self.node.create_subscription(Odometry, 'odom', self._odom_callback, qos_profile, callback_group=subscriber_callback_group)
        self._ir_intensity = self.node.create_subscription(IrIntensityVector, 'ir_intensity', self._ir_intensity_callback, qos_profile, callback_group=subscriber_callback_group)
        self._hazard_detection = self.node.create_subscription(HazardDetectionVector, 'hazard_detection', self._hazard_detection_callback, qos_profile, callback_group=subscriber_callback_group)
        self._interface_buttons = self.node.create_subscription(InterfaceButtons, 'interface_buttons', self._interface_buttons_callback, qos_profile, callback_group=subscriber_callback_group)
        self._battery_state = self.node.create_subscription(BatteryState, 'battery_state', self._battery_state_callback, qos_profile, callback_group=subscriber_callback_group)
        self._imu = self.node.create_subscription(Imu, 'imu', self._imu_callback, qos_profile, callback_group=subscriber_callback_group)
        self._dock_status = self.node.create_subscription(DockStatus, 'dock_status', self._dock_status_callback, qos_profile, callback_group=subscriber_callback_group)
        self._ir_opcode = self.node.create_subscription(IrOpcode, 'ir_opcode', self._ir_opcode_callback, qos_profile, callback_group=subscriber_callback_group)

        # Add topics to debugger
        self.debug.subscriptions = [self._odom, self._ir_intensity, self._hazard_detection, self._interface_buttons, self._battery_state, self._imu, self._dock_status, self._ir_opcode]

    def get_ir_proximity(self):
        """Get most recent IR proximity sensor values as a list of 7 integers."""
        return self._subscription_msgs.ir_values
        
    def get_position(self) -> Position:
        """Get robot's position and heading.
        
        Units:
            x, y: cm
            heading: deg
        """
        return self._subscription_msgs.position # return position
    
    def get_bumpers(self) -> HazardBumper:
        """Returns object of most recently seen bumper states."""
        return self._subscription_msgs.bumpers
    
    def get_cliff_sensors(self) -> HazardCliff:
        """Returns object of most recently seen cliff sensor states."""
        return self._subscription_msgs.cliff
    
    def get_touch_sensors(self) -> InterfaceButtons:
        """Returns object of most recently seen touch sensor states."""
        return self._subscription_msgs.buttons
    
    def get_battery_level(self) -> int | float:
        """Get battery level as percentage."""
        return self._subscription_msgs.battery
    
    def get_accelerometer(self) -> Acceleration:
        """Get accelerometer values as an object with x, y, and z properties."""
        return self._subscription_msgs.acceleration
    
    def get_docking_values(self) -> DockingValues:
        """Get most recent docking values as an object with is_docked, dock_visible, sensor, greenBuoy, redBuoy, and forceField properties."""
        return self._subscription_msgs.dockingValues