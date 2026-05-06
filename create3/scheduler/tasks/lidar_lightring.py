from typing import TYPE_CHECKING

from create3.models.common import Nodes
from create3 import RobotNode, CompanionNode

if TYPE_CHECKING:
    from create3.scheduler import TaskScheduler

def lidar_lightring_task(scheduler: "TaskScheduler") -> None:
    """Update the robot's lightring LEDs based on the closest LiDAR obstacle."""
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)

    if not companion.get_scans():
        return

    robot.set_lights(companion.tools.lidar.get_motion_lightring(companion.get_scans()))
