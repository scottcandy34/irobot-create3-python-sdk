#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from sensor_msgs.msg import LaserScan, Range

if TYPE_CHECKING:
    from create3.ros.companion import Subscriber

def scan_callback(subscriber: "Subscriber", scan: LaserScan) -> None:
    """Handle incoming LiDAR LaserScan data and update the shared lidar state.

    Converts all distance values from meters to centimeters and all angles
    from radians to degrees to match the internal units used throughout the SDK.
    """
    subscriber.update_uptime(subscriber._scan.topic_name)

    if not scan.ranges:
        return

    # Convert ranges from meters → cm (preserve None/inf for invalid rays)
    subscriber._subscription_msgs.lidar.ranges = [(r * 100.0 if isinstance(r, float) else None) for r in scan.ranges]

    subscriber._subscription_msgs.lidar.scan_time = scan.scan_time

    # Convert angles to degrees
    subscriber._subscription_msgs.lidar.angle_max = math.degrees(scan.angle_max)
    subscriber._subscription_msgs.lidar.angle_min = math.degrees(scan.angle_min)
    subscriber._subscription_msgs.lidar.angle_increment = math.degrees(scan.angle_increment)

    # Convert range limits to cm
    subscriber._subscription_msgs.lidar.range_max = scan.range_max * 100.0
    subscriber._subscription_msgs.lidar.range_min = scan.range_min * 100.0

    subscriber._subscription_msgs.lidar.time_increment = scan.time_increment

def range_callback(subscriber: "Subscriber", range_: Range) -> None:
    """Handle incoming ultrasonic Range data and update the shared ultrasonic state.

    Converts all distance values from meters to centimeters to match the
    internal units used throughout the SDK.
    """
    subscriber.update_uptime(subscriber._range.topic_name)

    subscriber._subscription_msgs.ultrasonic.field_of_view = range_.field_of_view

    # Convert distances to cm
    subscriber._subscription_msgs.ultrasonic.max_range = range_.max_range * 100.0
    subscriber._subscription_msgs.ultrasonic.min_range = range_.min_range * 100.0
    subscriber._subscription_msgs.ultrasonic.range = range_.range * 100.0