#
# Companion Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import pprint as _pprint

class Lidar():
    """Stores companion lidar values."""
    angle_min: float = 0.0 # start angle of scan
    angle_max: float = 0.0 # end angle of scan
    angle_increment: float = 0.0 # angular distance between measurements
    range_min: float = 0.0 # minimum range value
    range_max: float = 0.0 # maximum range value
    time_increment: float = 0.0 # rime between measurements
    scan_time: float = 0.0 # time between scans
    ranges: list[float] = []

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)

class Ultrasonic():
    """Stores companion ultrasonic sensor values."""
    field_of_view: float = 0.0
    min_range: float = 0.0
    max_range: float = 0.0
    range: float = 0.0

    def __str__(self):
        return _pprint.pformat(self, indent = 4, width = 80)
