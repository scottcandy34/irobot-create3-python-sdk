#
# Companion Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

@dataclass
class Lidar:
    """Stores the latest LiDAR scan data from the companion node.

    All distances are in centimeters and all angles are in degrees
    (converted from the raw ROS LaserScan message).
    """

    angle_min: float = 0.0          # start angle of scan (degrees)
    angle_max: float = 0.0          # end angle of scan (degrees)
    angle_increment: float = 0.0    # angular step between measurements (degrees)
    range_min: float = 0.0          # minimum measurable range (cm)
    range_max: float = 0.0          # maximum measurable range (cm)
    time_increment: float = 0.0     # time between consecutive measurements (s)
    scan_time: float = 0.0          # time to complete one full scan (s)

    # List of range measurements (cm). Invalid rays are usually None or inf.
    ranges: list[float] = field(default_factory=list)

    def size(self) -> int:
        """Return the number of range measurements in the current scan."""
        return len(self.ranges)

    def get_range(self, start: int, end: int) -> list[float]:
        """Return a slice of the range array, handling wrap-around."""
        if start < end:
            return self.ranges[start:end]
        return self.ranges[start:] + self.ranges[:end]

    def get_front_slice(self) -> list[float]:
        """Return the forward-facing slice (≈60° wide) of the LiDAR scan."""
        n = self.size()
        slice_width = n // 12
        center = n // 2

        start = (center - slice_width) % n
        end = (center + slice_width) % n
        return self.get_range(start, end)

    def get_left_slice(self) -> list[float]:
        """Return the left-side slice (≈60° wide) of the LiDAR scan."""
        n = self.size()
        slice_width = n // 12
        center = (3 * n) // 4

        start = (center - slice_width) % n
        end = (center + slice_width) % n
        return self.get_range(start, end)

    def get_right_slice(self) -> list[float]:
        """Return the right-side slice (≈60° wide) of the LiDAR scan.

        Used for right-hand wall following.
        """
        n = self.size()
        slice_width = n // 12
        center = n // 4

        start = (center - slice_width) % n
        end = (center + slice_width) % n
        return self.get_range(start, end)

@dataclass
class Wall:
    """Stores information about a single detected straight wall segment."""

    length: float = 0.0
    xmin: float = 0.0
    xmax: float = 0.0
    slope: float = 0.0
    intercept: float = 0.0

@dataclass
class Column:
    """Stores information about a single detected circular column/obstacle."""

    cx: float
    cy: float
    radius: float
    start_angle: float
    end_angle: float
    arc_length: float

@dataclass
class Detections:
    """Container for all detected geometric features (walls + columns)."""

    columns: list[Column] = field(default_factory=list)
    walls: list[Wall] = field(default_factory=list)

@dataclass
class Interaction:
    """Stores the result of a collision check between the robot and a wall."""

    in_path: bool = False
    distance: float = 0.0
    angle: float = 0.0

@dataclass
class Ultrasonic:
    """Stores the latest ultrasonic (Range) sensor data."""

    field_of_view: float = 0.0
    min_range: float = 0.0
    max_range: float = 0.0
    range: float = 0.0
    