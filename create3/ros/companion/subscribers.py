#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

from create3.models.common import Stamped
from create3.utils.common.other import TIMEOUT
from create3.utils import Logger, MonitoredSubscription
from create3.models.companion import Subscribe, Topics, Ultrasonic, Lidar

from .callbacks import (
    scan_callback,
    range_callback
)

qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(Logger):
    """ROS subscriber manager for companion data (LiDAR and ultrasonic).

    Handles the `/scan` (LaserScan) and `/range` (Range) topics.
    All callbacks run inside a mutually exclusive callback group so they never
    block each other. The class also registers itself with the debugger for
    uptime and interface monitoring.
    """

    def __init__(self, node: Node) -> None:
        """Initialize subscriptions for LiDAR scan and ultrasonic range data.

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
            
        return None
    
    def wait(self, name: Topics):
        if not self.find(name).ready_event.wait(TIMEOUT):
            self.print_warning(f"Timeout waiting for first message on {name}")
    
    @property
    def lidar(self) -> Stamped[Lidar]:
        if not self.find(Topics.SCAN):
            self.topics.append(self.node.create_monitored_subscription(LaserScan, Topics.SCAN, lambda msg: scan_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.SCAN)
        return self.msgs.lidar
    
    @lidar.setter
    def lidar(self, msg: Stamped[Lidar]):
        self.msgs.lidar = msg
        
    @property
    def ultrasonic(self) -> Stamped[Ultrasonic]:
        if not self.find(Topics.RANGE):
            self.topics.append(self.node.create_monitored_subscription(Range, Topics.RANGE, lambda msg: range_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.RANGE)
        return self.msgs.ultrasonic
    
    @ultrasonic.setter
    def ultrasonic(self, msg: Stamped[Ultrasonic]):
        self.msgs.ultrasonic = msg
