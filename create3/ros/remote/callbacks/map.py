#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid

from create3.models.common import Position, Stamped
from create3.models.remote import Map
from create3.utils import remote as tools
from create3.utils.common.coords import convert_to_euler

if TYPE_CHECKING:
    from create3.ros.remote import Subscriber

def map_callback(subscriber: "Subscriber", grid: OccupancyGrid) -> None:
    """Handle incoming occupancy grid (map) data and update the shared map state.

    Converts the grid origin from meters to centimeters and extracts the yaw
    angle from the quaternion orientation.
    """
    map_ = Map()

    map_.resolution = grid.info.resolution
    map_.grid = tools.slam.occupancy_grid_to_2d(grid)

    # Origin pose (converted to cm and degrees)
    origin_pos = grid.info.origin.position
    origin_orient = grid.info.origin.orientation

    position = Position()
    position.x = origin_pos.x * 100.0
    position.y = origin_pos.y * 100.0
    euler = convert_to_euler(origin_orient.x, origin_orient.y, origin_orient.z, origin_orient.w)
    position.angle = math.degrees(euler.yaw_z)

    map_.origin = position
    
    subscriber.map = Stamped(map_, Time.from_msg(grid.header.stamp))
