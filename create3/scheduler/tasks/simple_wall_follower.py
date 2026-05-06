from typing import TYPE_CHECKING

from create3.models.common import Nodes
from create3 import RobotNode, CompanionNode

if TYPE_CHECKING:
    from create3.scheduler import TaskScheduler

def simple_wall_follower_task(scheduler: "TaskScheduler") -> None:
    """Simple reactive wall-follower using LiDAR and PID control.

    Sends velocity commands directly to the robot.
    """
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)

    if not companion.get_scans():
        return

    lidar = companion.subscriber.lidar.data

    twist_msg = companion.tools.wall_follow.pid_lidar_to_twist(lidar)
    robot.publisher.send_velocity(twist_msg)
