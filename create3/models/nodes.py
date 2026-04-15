from enum import Enum, auto

class Nodes(Enum):
    CREATE3_ROBOT = auto()
    """Control the Create3 using the robot's built in interface."""
    CREATE3_COMPANION = auto()
    """Control the Create3 using a companion computer interface."""
    CREATE3_REMOTE = auto()
    """Control the Create3 using a remote control interface."""
    ROS_DEBUGGER = auto()
    """Watch the ROS interfaces of attached nodes and print warnings or errors if they are not working as expected."""
    TASK_SCHEDULAR = auto()
    """Schedular for running tasks on attached nodes and devices."""