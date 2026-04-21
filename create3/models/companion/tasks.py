#
# Companion Tasks for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from enum import StrEnum, auto

class Tasks(StrEnum):
    GENERATE_COORDS = auto()
    """Generate coordinates for detected walls and columns."""
    WALL_DETECTION = auto()
    """Detect walls using Lidar data."""
    COLUMN_DETECTION = auto()
    """Detect columns using Lidar data."""
    LIDAR_LIGHTRING = auto()
    """Use Lidar data to create a light ring pattern on the robot's lights."""
    SIMPLE_WALL_FOLLOWER = auto()
    """A simple wall following behavior using PID control based on Lidar data."""