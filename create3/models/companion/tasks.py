from enum import Enum, auto

class Tasks(Enum):
    GENERATE_COORDS = auto()
    """Generate coordinates for detected walls and columns."""
    WALL_DETECTION = auto()
    """Detect walls using Lidar data."""
    COLUMN_DETECTION = auto()
    """Detect columns using Lidar data."""