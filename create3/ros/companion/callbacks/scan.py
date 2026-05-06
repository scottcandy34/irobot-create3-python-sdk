#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from create3.models.common import Stamped
from create3.models.companion import Lidar

if TYPE_CHECKING:
    from create3.ros.companion import Subscriber

def scan_callback(subscriber: "Subscriber", scan: LaserScan) -> None:
    """Handle incoming LiDAR LaserScan data and update the shared lidar state.

    Converts all distance values from meters to centimeters and all angles
    from radians to degrees to match the internal units used throughout the SDK.
    """
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
    
    subscriber.lidar = Stamped(lidar, Time.from_msg(scan.header.stamp))
