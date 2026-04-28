#
# Common Tasks for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from enum import StrEnum, auto

class Tasks(StrEnum):
    """General/shared tasks that can run on any node type (robot, companion, or remote)."""

    HISTORY_KEEPER = auto()
    """Maintains a rolling history of point clouds, detections, or other temporal data
    for use in filtering, prediction, or multi-frame processing."""