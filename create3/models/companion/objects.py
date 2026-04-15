#
# Companion Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

@dataclass
class Lidar:
    """Stores companion lidar values."""
    angle_min: float = 0.0 # start angle of scan
    angle_max: float = 0.0 # end angle of scan
    angle_increment: float = 0.0 # angular distance between measurements
    range_min: float = 0.0 # minimum range value
    range_max: float = 0.0 # maximum range value
    time_increment: float = 0.0 # rime between measurements
    scan_time: float = 0.0 # time between scans
    ranges: list[float] = field(default_factory=list) # list of range measurements

    def size(self) -> int:
        """Returns the number of measurements in the scan."""
        return len(self.ranges)

@dataclass
class Wall:
    """Stores information about a detected wall."""
    length: float = 0.0
    xmin: float = 0.0
    xmax: float = 0.0
    slope: float = 0.0
    intercept: float = 0.0

@dataclass
class WallInteraction:
    """
    Stores information about the interaction between the robot and a wall, including whether the wall 
    is in the robot's path, the distance to the wall, and the angle between the robot's heading and the wall.
    """
    in_path: bool = False
    distance: float = 0.0
    angle: float = 0.0

@dataclass
class DetectedShapes:
    """Stores information about all detected walls."""
    walls: list[Wall] = field(default_factory=list)
    coords: list[tuple[float, float]] = field(default_factory=list)
    interactions: list[WallInteraction] = field(default_factory=list)

@dataclass
class Ultrasonic:
    """Stores companion ultrasonic sensor values."""
    field_of_view: float = 0.0
    min_range: float = 0.0
    max_range: float = 0.0
    range: float = 0.0
