#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from sensor_msgs.msg import LaserScan, Range

from create3.utils import Threading
from create3.models import SubscriberTopics

class MsgCallbacks(Threading if TYPE_CHECKING else object):
    """Handles incoming messages for subscribed topics."""

    def __init__(self, node):
        super().__init__(node)

        # Hidden global callback information
        self._subscription_msgs = SubscriberTopics.RPI
        """Contains the most recent messages received for each topic. Updated when a callback is triggered."""

    def _scan_callback(self, scan: LaserScan):
        self.update_uptime(self._scan.topic_name)

        if scan.ranges:
            self._subscription_msgs.lidar.ranges = [(range_ * 100 if isinstance(range_, float) else None) for range_ in scan.ranges]
            self._subscription_msgs.lidar.scan_time = scan.scan_time
            
            self._subscription_msgs.lidar.angle_max = math.degrees(scan.angle_max)
            self._subscription_msgs.lidar.angle_min = math.degrees(scan.angle_min)
            self._subscription_msgs.lidar.angle_increment = math.degrees(scan.angle_increment)
            
            self._subscription_msgs.lidar.range_max = scan.range_max * 100
            self._subscription_msgs.lidar.range_min = scan.range_min * 100
            self._subscription_msgs.lidar.time_increment = scan.time_increment
            
    def _range_callback(self, range_: Range):
        self.update_uptime(self._range.topic_name)

        self._subscription_msgs.ultrasonic.field_of_view = range_.field_of_view
        self._subscription_msgs.ultrasonic.max_range = range_.max_range * 100
        self._subscription_msgs.ultrasonic.min_range = range_.min_range * 100
        self._subscription_msgs.ultrasonic.range = range_.range * 100
