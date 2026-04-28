#
# Companion Tasks for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from enum import StrEnum, auto

class Tasks(StrEnum):
    """Tasks that run on the companion computer node."""

    GENERATE_COORDS = auto()
    """Convert latest LiDAR scan into a world-frame point cloud (x, y coordinates)."""

    WALL_DETECTION = auto()
    """Detect straight wall segments using RANSAC on the point cloud."""

    COLUMN_DETECTION = auto()
    """Detect circular columns/obstacles using RANSAC on the point cloud."""

    LIDAR_LIGHTRING = auto()
    """Use LiDAR data to create a directional lightring pattern on the robot's LEDs."""

    SIMPLE_WALL_FOLLOWER = auto()
    """Reactive wall-following behavior using LiDAR and PID control."""