#
# Subscriber Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from sensor_msgs.msg import Joy
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from yolo_msgs.msg import DetectionArray

from create3.models.common import Stamped
from create3.utils.common.other import TIMEOUT
from create3.utils import Logger, MonitoredSubscription
from create3.models.remote import Controller, Map, Yolo, Subscribe, Topics

from .callbacks import (
    joy_callback,
    map_callback,
    yolo_detections_callback
)

qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class Subscriber(Logger):
    """ROS subscriber manager for companion/remote topics (joystick, map, YOLO).

    This class handles higher-level perception and control input topics.
    All callbacks run in a mutually exclusive callback group so they never
    interfere with each other or with other nodes.

    The class also registers itself with the watchdog for uptime and
    interface monitoring.
    """

    def __init__(self, *args, **kwargs) -> None:
        """Initialize subscriptions for joystick input, occupancy grid, and YOLO detections.

        Parameters
        ----------
        node : Node
            The ROS node that owns these subscriptions.
        """
        super().__init__(*args, **kwargs)

        # Shared container that holds the latest message data for every topic
        self.msgs: Subscribe = Subscribe()

        # Use a mutually exclusive callback group so callbacks never block each other
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Register all subscriptions with the watchdog for uptime monitoring
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
    def controller(self) -> Controller:
        if not self.find(Topics.JOY):
            self.topics.append(self.node.create_monitored_subscription(Joy, Topics.JOY, lambda msg: joy_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.JOY)
        return self.msgs.controller
    
    @controller.setter
    def controller(self, msg: Controller):
        self.msgs.controller = msg
        
    @property
    def map(self) -> Stamped[Map]:
        if not self.find(Topics.MAP):
            self.topics.append(self.node.create_monitored_subscription(OccupancyGrid, Topics.MAP, lambda msg: map_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.MAP)
        return self.msgs.map
    
    @map.setter
    def map(self, msg: Stamped[Map]):
        self.msgs.map = msg
        
    @property
    def yolo(self) -> Stamped[Yolo]:
        if not self.find(Topics.YOLO_DETECTIONS):
            self.topics.append(self.node.create_monitored_subscription(DetectionArray, Topics.YOLO_DETECTIONS, lambda msg: yolo_detections_callback(self, msg), qos_profile, callback_group=self.callback_group))
            self.wait(Topics.YOLO_DETECTIONS)
        return self.msgs.yolo
    
    @yolo.setter
    def yolo(self, msg: Stamped[Yolo]):
        self.msgs.yolo = msg
