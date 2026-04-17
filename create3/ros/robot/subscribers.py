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

from create3.utils import Threading
from create3.models.common import Position
from create3.models.robot import HazardBumper, HazardCliff, Acceleration, DockingValues, Subscribe

from .callbacks.msg import (
    odom_callback,
    ir_intensity_callback,
    hazard_detection_callback,
    interface_buttons_callback,
    battery_state_callback,
    imu_callback,
    dock_status_callback,
    ir_opcode_callback
)

qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(Threading if TYPE_CHECKING else object):
    """Handles ROS subscribers for robot data."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Hidden global callback information
        self._subscription_msgs = Subscribe()
        """Contains the most recent messages received for each topic. Updated when a callback is triggered."""

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()
    
        # Create Subscription
        self._odom = self.node.create_subscription(Odometry, 'odom', lambda msg: odom_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._ir_intensity = self.node.create_subscription(IrIntensityVector, 'ir_intensity', lambda msg: ir_intensity_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._hazard_detection = self.node.create_subscription(HazardDetectionVector, 'hazard_detection', lambda msg: hazard_detection_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._interface_buttons = self.node.create_subscription(InterfaceButtons, 'interface_buttons', lambda msg: interface_buttons_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._battery_state = self.node.create_subscription(BatteryState, 'battery_state', lambda msg: battery_state_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._imu = self.node.create_subscription(Imu, 'imu', lambda msg: imu_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._dock_status = self.node.create_subscription(DockStatus, 'dock_status', lambda msg: dock_status_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._ir_opcode = self.node.create_subscription(IrOpcode, 'ir_opcode', lambda msg: ir_opcode_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)

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