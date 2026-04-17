from typing import TYPE_CHECKING

from create3.models import Nodes
from create3.models.robot import Tasks
from create3 import RobotNode, CompanionNode, RemoteNode

if TYPE_CHECKING:
    from create3.schedular import TaskSchedular

def ir_lightring_task(scheduler: "TaskSchedular"):
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)
    if not robot.get_ir_proximity():
        return
    
    robot.set_lights(robot.tools.ir.get_motion_lightring(robot.get_ir_proximity()))