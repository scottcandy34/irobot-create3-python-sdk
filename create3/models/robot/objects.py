#
# Robot Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

@dataclass
class HazardBumper:
    """Stores the current state of all bumper sensors on the robot."""

    right: bool = False
    front_right: bool = False
    front_center: bool = False
    front_left: bool = False
    left: bool = False

@dataclass
class HazardCliff:
    """Stores the current state of all cliff sensors on the robot."""

    side_right: bool = False
    front_right: bool = False
    front_left: bool = False
    side_left: bool = False

@dataclass
class Acceleration:
    """Stores the latest linear acceleration readings from the IMU."""

    x: int | float = 0.0
    y: int | float = 0.0
    z: int | float = 0.0

@dataclass
class DockingValues:
    """Stores the latest docking-related sensor values."""

    is_docked: bool = False
    dock_visible: bool = False
    sensor: int = 0
    greenBuoy: bool = False
    redBuoy: bool = False
    forceField: bool = False

@dataclass
class RobotButtons:
    """Stores the current state of the physical buttons on the robot."""

    button_1: bool = False
    button_power: bool = False
    button_2: bool = False