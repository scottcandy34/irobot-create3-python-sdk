#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Range

from create3.models.common import Stamped
from create3.models.companion import Lidar, Ultrasonic

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
    
    lidar = Lidar()

    # Convert ranges from meters → cm (preserve None/inf for invalid rays)
    lidar.ranges = [(r * 100.0 if isinstance(r, float) else None) for r in scan.ranges]

    lidar.scan_time = scan.scan_time

    # Convert angles to degrees
    lidar.angle_max = math.degrees(scan.angle_max)
    lidar.angle_min = math.degrees(scan.angle_min)
    lidar.angle_increment = math.degrees(scan.angle_increment)

    # Convert range limits to cm
    lidar.range_max = scan.range_max * 100.0
    lidar.range_min = scan.range_min * 100.0

    lidar.time_increment = scan.time_increment
    
    subscriber._subscription_msgs.lidar = Stamped(lidar, Time.from_msg(scan.header.stamp))

def range_callback(subscriber: "Subscriber", range_: Range) -> None:
    """Handle incoming ultrasonic Range data and update the shared ultrasonic state.

    Converts all distance values from meters to centimeters to match the
    internal units used throughout the SDK.
    """
    subscriber.update_uptime(subscriber._range.topic_name)
    
    ultrasonic = Ultrasonic()

    ultrasonic.field_of_view = range_.field_of_view

    # Convert distances to cm
    ultrasonic.max_range = range_.max_range * 100.0
    ultrasonic.min_range = range_.min_range * 100.0
    ultrasonic.range = range_.range * 100.0
    
    subscriber._subscription_msgs.ultrasonic = Stamped(ultrasonic, Time.from_msg(range_.header.stamp))
    