#
# Map Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""Tools for working with occupancy grid maps, including converting a ROS OccupancyGrid message to a 2D NumPy array."""

import numpy as np
from nav_msgs.msg import OccupancyGrid

def occupancy_grid_to_2d(grid: OccupancyGrid) -> np.ndarray:
    """Convert a ROS OccupancyGrid message into a 2D NumPy array.

    The resulting array uses the standard ROS occupancy conventions:
      • -1  = unknown
      •  0  = free space
      • 100 = occupied

    The array shape is (height, width) to match the ROS grid coordinate system
    (row-major order, origin at bottom-left).

    Parameters
    ----------
    grid : OccupancyGrid
        ROS message containing the occupancy grid data and metadata.

    Returns
    -------
    np.ndarray
        2D array of dtype int8 with shape (height, width).
        Returns a (0, 0) empty array if the grid data is empty.
    """
    if not grid.data:
        # Empty grid → return properly shaped empty array
        return np.array([], dtype=np.int8).reshape(0, 0)

    # Convert flat list to 2D array using the grid metadata
    data = np.array(grid.data, dtype=np.int8).reshape((grid.info.height, grid.info.width))

    return data