#
# Map Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with occupancy grid maps, including converting a ROS OccupancyGrid message to a 2D NumPy array."""

import numpy as np
from nav_msgs.msg import OccupancyGrid

def occupancy_grid_to_2d(grid: OccupancyGrid) -> np.ndarray:
    """
    Convert a ROS OccupancyGrid message to a 2D NumPy array of int8 values, where -1 is unknown, 
    0 is free, and 100 is occupied. Returns an empty array if the grid data is empty.
    """
    if not grid.data:
        return np.array([]).reshape(0,0)

    data = np.array(grid.data, dtype=np.int8).reshape((grid.info.height, grid.info.width))
    return data