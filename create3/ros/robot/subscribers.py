#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.node import Node
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
    """ROS subscriber manager for the iRobot Create3.

    Creates subscriptions to all core robot topics (odometry, IR intensity,
    hazards, buttons, battery, IMU, dock status, IR opcode, etc.) and keeps
    the most recent data in a shared `_subscription_msgs` container.

    All callbacks run inside a `MutuallyExclusiveCallbackGroup` so they never
    block each other. The class also registers itself with the debugger for
    uptime and interface monitoring.
    """

    def __init__(self, node: Node) -> None:
        """Initialize the subscriber and create all ROS topic subscriptions.

        Parameters
        ----------
        node : Node
            The ROS node that owns these subscriptions.
        """
        super().__init__(node)  # initialize Threading + Logger

        # Shared container that holds the latest message data for every topic
        self._subscription_msgs: Subscribe = Subscribe()

        # Use a mutually exclusive callback group so callbacks never block each other
        subscriber_callback_group = MutuallyExclusiveCallbackGroup()

        # ------------------------------------------------------------------
        # Create all subscriptions
        # ------------------------------------------------------------------
        self._odom = self.node.create_subscription(Odometry, 'odom', lambda msg: odom_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._ir_intensity = self.node.create_subscription(IrIntensityVector, 'ir_intensity', lambda msg: ir_intensity_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._hazard_detection = self.node.create_subscription(HazardDetectionVector, 'hazard_detection', lambda msg: hazard_detection_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._interface_buttons = self.node.create_subscription(InterfaceButtons, 'interface_buttons', lambda msg: interface_buttons_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._battery_state = self.node.create_subscription(BatteryState, 'battery_state', lambda msg: battery_state_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._imu = self.node.create_subscription(Imu, 'imu', lambda msg: imu_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._dock_status = self.node.create_subscription(DockStatus, 'dock_status', lambda msg: dock_status_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)
        self._ir_opcode = self.node.create_subscription(IrOpcode, 'ir_opcode', lambda msg: ir_opcode_callback(self, msg), qos_profile, callback_group=subscriber_callback_group)


        # Register all subscriptions with the debugger for uptime monitoring
        self.debug.subscriptions = [
            self._odom,
            self._ir_intensity,
            self._hazard_detection,
            self._interface_buttons,
            self._battery_state,
            self._imu,
            self._dock_status,
            self._ir_opcode,
        ]

    # ----------------------------------------------------------------------
    # Public getters (convenience API for the rest of the codebase)
    # ----------------------------------------------------------------------

    def get_ir_proximity(self) -> list[int]:
        """Return the most recent IR proximity sensor readings (7 integers)."""
        return self._subscription_msgs.ir_values

    def get_position(self) -> Position:
        """Return the robot's current position and heading.

        Units:
            x, y     → centimeters
            angle    → degrees
        """
        return self._subscription_msgs.position

    def get_bumpers(self) -> HazardBumper:
        """Return the most recent bumper states as a `HazardBumper` object."""
        return self._subscription_msgs.bumpers

    def get_cliff_sensors(self) -> HazardCliff:
        """Return the most recent cliff sensor states as a `HazardCliff` object."""
        return self._subscription_msgs.cliff

    def get_touch_sensors(self) -> InterfaceButtons:
        """Return the most recent physical button states."""
        return self._subscription_msgs.buttons

    def get_battery_level(self) -> int | float:
        """Return the current battery level as a percentage (0–100)."""
        return self._subscription_msgs.battery

    def get_accelerometer(self) -> Acceleration:
        """Return the most recent linear acceleration values (x, y, z)."""
        return self._subscription_msgs.acceleration

    def get_docking_values(self) -> DockingValues:
        """Return the most recent docking sensor values
        (is_docked, dock_visible, sensor, redBuoy, greenBuoy, forceField)."""
        return self._subscription_msgs.dockingValues