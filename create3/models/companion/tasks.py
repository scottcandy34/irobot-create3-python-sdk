from enum import Enum, auto

class Tasks(Enum):
    WALL_DETECTION = auto()
    """Detect walls using Lidar data."""
    COLUMN_DETECTION = auto()
    """Detect collumns using Lidar data."""