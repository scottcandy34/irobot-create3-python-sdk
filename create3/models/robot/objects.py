#
# Robot Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import pprint as _pprint

class Position():
    """Stores robot position values."""
    x: int | float = 0.0
    y: int | float = 0.0
    angle: int | float = 0.0

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)

class HazardBumper():
    """Stores robot bumper values."""
    right: bool = False
    front_right: bool = False
    front_center: bool = False
    front_left: bool = False
    left: bool = False

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)
    
class HazardCliff():
    """Stores robot cliff sensor values."""
    side_right: bool = False
    front_right: bool = False
    front_left: bool = False
    side_left: bool = False

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)
    
class Acceleration():
    """Stores robot acceleration values."""
    x: int | float = 0.0
    y: int | float = 0.0
    z: int | float = 0.0

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)
    
class DockingValues():
    """Stores robot docking values."""
    is_docked = False
    dock_visible = False
    sensor: int = 0
    greenBuoy = False
    redBuoy = False
    forceField = False

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)
    
class RobotButtons():
    """Stores robot button pressed values."""
    button_1 = False
    button_power = False
    button_2 = False

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)
