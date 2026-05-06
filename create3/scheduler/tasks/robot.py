from typing import TYPE_CHECKING

from create3.models.common import Nodes
from create3.models.robot import Tasks
from create3 import RobotNode, CompanionNode, RemoteNode

if TYPE_CHECKING:
    from create3.scheduler import TaskScheduler

def ir_lightring_task(scheduler: "TaskScheduler") -> None:
    """Update the robot's lightring LEDs based on the strongest IR proximity signal.

    Uses the IR sensors on the robot to create a directional "spotlight"
    effect on the 6-LED ring — the same visual feedback used by the LiDAR
    lightring task, but driven by IR instead.
    """
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)

    # No IR data available → do nothing
    if not robot.get_ir_proximity():
        return

    # Generate and apply the lightring pattern
    robot.set_lights(robot.tools.ir.get_motion_lightring(robot.get_ir_proximity()))