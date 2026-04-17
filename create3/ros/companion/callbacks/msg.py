#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from sensor_msgs.msg import LaserScan, Range

if TYPE_CHECKING:
    from create3.ros.companion import Subscriber

def scan_callback(subscriber: "Subscriber", scan: LaserScan):
    subscriber.update_uptime(subscriber._scan.topic_name)

    if scan.ranges:
        subscriber._subscription_msgs.lidar.ranges = [(range_ * 100 if isinstance(range_, float) else None) for range_ in scan.ranges]
        subscriber._subscription_msgs.lidar.scan_time = scan.scan_time
        
        subscriber._subscription_msgs.lidar.angle_max = math.degrees(scan.angle_max)
        subscriber._subscription_msgs.lidar.angle_min = math.degrees(scan.angle_min)
        subscriber._subscription_msgs.lidar.angle_increment = math.degrees(scan.angle_increment)
        
        subscriber._subscription_msgs.lidar.range_max = scan.range_max * 100
        subscriber._subscription_msgs.lidar.range_min = scan.range_min * 100
        subscriber._subscription_msgs.lidar.time_increment = scan.time_increment
        
def range_callback(subscriber: "Subscriber", range_: Range):
    subscriber.update_uptime(subscriber._range.topic_name)

    subscriber._subscription_msgs.ultrasonic.field_of_view = range_.field_of_view
    subscriber._subscription_msgs.ultrasonic.max_range = range_.max_range * 100
    subscriber._subscription_msgs.ultrasonic.min_range = range_.min_range * 100
    subscriber._subscription_msgs.ultrasonic.range = range_.range * 100
