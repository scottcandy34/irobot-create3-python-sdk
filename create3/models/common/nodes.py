#
# Node Names for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from enum import StrEnum, auto

class Nodes(StrEnum):
    """Enumeration of all ROS node names used throughout the iRobot Create3 SDK.

    These names are used when creating nodes and for logging/debugging purposes.
    """

    CREATE3_ROBOT = auto()
    """Main robot node — provides direct low-level access to the Create3 hardware
    (sensors, actuators, actions, services)."""

    CREATE3_COMPANION = auto()
    """Companion computer node — runs on the Raspberry Pi (or similar) and handles
    LiDAR, ultrasonic, vision, and advanced perception tasks."""

    CREATE3_REMOTE = auto()
    """Remote control node — runs on a laptop or computer and handles joystick
    input, high-level commands, and remote operation."""

    ROS_WATCHDOG = auto()
    """Global watchdog node — monitors all attached nodes and logs warnings
    or errors when ROS interfaces are missing or misbehaving."""

    TASK_SCHEDULER = auto()
    """Task scheduler node — manages periodic tasks across all registered devices
    and nodes."""