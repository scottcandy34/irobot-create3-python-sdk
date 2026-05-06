#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, InterfaceButtons, DockStatus, IrOpcode

from create3.utils.common.other import TIMEOUT
from create3.utils import Logger, MonitoredSubscription, Node
from create3.models.common import Position, Stamped
from create3.models.robot import HazardBumper, HazardCliff, Acceleration, DockingValues, Subscribe, RobotButtons, Topics

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

class Subscriber(Logger):
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
        self.msgs: Subscribe = Subscribe()

        # Use a mutually exclusive callback group so callbacks never block each other
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Register all subscriptions with the debugger for uptime monitoring
        self.topics: list[MonitoredSubscription] = []
        
    def find(self, name: Topics) -> MonitoredSubscription:
        for subscription in self.topics:
            if name == subscription.topic_name:
                return subscription
            
        self.print_error(f'Cannot find topic {name}')
        return None
    
    def wait(self, name: Topics):
        if not self.find(name).ready_event.wait(TIMEOUT):
            self.print_warning(f"Timeout waiting for first message on {name}")
    
    @property
    def position(self) -> Stamped[Position]:
        if not self.find(Topics.ODOM):
            self.topics.append(self.node.create_monitored_subscription(Odometry, Topics.ODOM, lambda msg: odom_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.ODOM)
        return self.msgs.position
    
    @position.setter
    def position(self, msg: Stamped[Position]):
        self.msgs.position = msg
    
    @property
    def ir_values(self) -> Stamped[list[int]]:
        if not self.find(Topics.IR_INTENSITY):
            self.topics.append(self.node.create_monitored_subscription(IrIntensityVector, Topics.IR_INTENSITY, lambda msg: ir_intensity_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.IR_INTENSITY)
        return self.msgs.ir_values
    
    @ir_values.setter
    def ir_values(self, msg: Stamped[list[int]]):
        self.msgs.ir_values = msg
    
    @property
    def bumpers(self) -> HazardBumper:
        if not self.find(Topics.HAZARD_DETECTION):
            self.topics.append(self.node.create_monitored_subscription(HazardDetectionVector, Topics.HAZARD_DETECTION, lambda msg: hazard_detection_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.HAZARD_DETECTION)
        return self.msgs.bumpers
    
    @bumpers.setter
    def bumpers(self, msg: HazardBumper):
        self.msgs.bumpers = msg
    
    @property
    def cliff_sensors(self) -> HazardCliff:
        if not self.find(Topics.HAZARD_DETECTION):
            self.topics.append(self.node.create_monitored_subscription(HazardDetectionVector, Topics.HAZARD_DETECTION, lambda msg: hazard_detection_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.HAZARD_DETECTION)
        return self.msgs.cliff
    
    @cliff_sensors.setter
    def cliff_sensors(self, msg: HazardCliff):
        self.msgs.cliff = msg
    
    @property
    def buttons(self) -> RobotButtons:
        if not self.find(Topics.INTERFACE_BUTTONS):
            self.topics.append(self.node.create_monitored_subscription(InterfaceButtons, Topics.INTERFACE_BUTTONS, lambda msg: interface_buttons_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.INTERFACE_BUTTONS)
        return self.msgs.buttons
    
    @buttons.setter
    def buttons(self, msg: RobotButtons):
        self.msgs.buttons = msg
    
    @property
    def battery(self) -> float:
        if not self.find(Topics.BATTERY_STATE):
            self.topics.append(self.node.create_monitored_subscription(BatteryState, Topics.BATTERY_STATE, lambda msg: battery_state_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.BATTERY_STATE)
        return self.msgs.battery
    
    @battery.setter
    def battery(self, msg: float):
        self.msgs.battery = msg
    
    @property
    def acceleration(self) -> Stamped[Acceleration]:
        if not self.find(Topics.IMU):
            self.topics.append(self.node.create_monitored_subscription(Imu, Topics.IMU, lambda msg: imu_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.IMU)
        return self.msgs.acceleration
    
    @acceleration.setter
    def acceleration(self, msg: Stamped[Acceleration]):
        self.msgs.acceleration = msg
    
    @property
    def docking_values(self) -> DockingValues:
        if not self.find(Topics.DOCK_STATUS):
            self.topics.append(self.node.create_monitored_subscription(DockStatus, Topics.DOCK_STATUS, lambda msg: dock_status_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.DOCK_STATUS)
        if not self.find(Topics.IR_OPCODE):
            self.topics.append(self.node.create_monitored_subscription(IrOpcode, Topics.IR_OPCODE, lambda msg: ir_opcode_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.IR_OPCODE)
        return self.msgs.docking_values
    
    @docking_values.setter
    def docking_values(self, msg: DockingValues):
        self.msgs.docking_values = msg
