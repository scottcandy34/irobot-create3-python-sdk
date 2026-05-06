#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from rclpy.time import Time
from nav_msgs.msg import Odometry

from create3.utils import common as tools
from create3.models.common import Position, Stamped

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def odom_callback(subscriber: "Subscriber", odom: Odometry) -> None:
    """Handle incoming odometry data and update the shared subscription state.

    Converts pose from meters to centimeters and quaternion to Euler yaw (degrees).
    """
    pos = odom.pose.pose.position
    orient = odom.pose.pose.orientation

    position = Position()
    position.x = pos.x * 100.0
    position.y = pos.y * 100.0
    euler = tools.coords.convert_to_euler(orient.x, orient.y, orient.z, orient.w)
    position.angle = math.degrees(euler.yaw_z)

    subscriber.position = Stamped(position, Time.from_msg(odom.header.stamp))
