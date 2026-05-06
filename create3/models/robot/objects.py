#
# Robot Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from dataclasses import dataclass, field

from create3.models.common import Button

@dataclass
class HazardBumper:
    """Stores the current state of all bumper sensors on the robot."""

    right: Button = field(default_factory=Button)
    front_right: Button = field(default_factory=Button)
    front_center: Button = field(default_factory=Button)
    front_left: Button = field(default_factory=Button)
    left: Button = field(default_factory=Button)

@dataclass
class HazardCliff:
    """Stores the current state of all cliff sensors on the robot."""

    side_right: Button = field(default_factory=Button)
    front_right: Button = field(default_factory=Button)
    front_left: Button = field(default_factory=Button)
    side_left: Button = field(default_factory=Button)

@dataclass
class Acceleration:
    """Stores the latest linear acceleration readings from the IMU."""

    x: int | float = 0.0
    y: int | float = 0.0
    z: int | float = 0.0

@dataclass
class DockingValues:
    """Stores the latest docking-related sensor values."""

    is_docked: Button = field(default_factory=Button)
    dock_visible: Button = field(default_factory=Button)
    sensor: int = 0
    greenBuoy: bool = False
    redBuoy: bool = False
    forceField: bool = False

@dataclass
class RobotButtons:
    """Stores the current state of the physical buttons on the robot."""

    button_1: Button = field(default_factory=Button)
    button_power: Button = field(default_factory=Button)
    button_2: Button = field(default_factory=Button)