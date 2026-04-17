from torch import TYPE_CHECKING

from create3.models import Nodes
from create3.models.companion import Tasks
from create3 import RobotNode, CompanionNode, RemoteNode

if TYPE_CHECKING:
    from create3.schedular import TaskSchedular

def generate_coords_task(scheduler: "TaskSchedular"):
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)
    
    scheduler._outputs[Tasks.GENERATE_COORDS] = [companion.tools.lidar.get_coords(companion.get_scans(), index, robot.get_position())for index in range(companion.get_scans().size())]

def wall_detection_task(scheduler: "TaskSchedular"):
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    coords: list[tuple[float, float]] = scheduler.get_task_output(Tasks.GENERATE_COORDS)
    if coords is None:
        return 
    
    scheduler._outputs[Tasks.WALL_DETECTION] = companion.tools.lidar.find_lines_and_segments([point for point in coords if point is not None])

def column_detection_task(scheduler: "TaskSchedular"):
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    coords: list[tuple[float, float]] = scheduler.get_task_output(Tasks.GENERATE_COORDS)
    if coords is None:
        return 
        
    scheduler._outputs[Tasks.COLUMN_DETECTION] = companion.tools.lidar.find_circles_and_arcs([point for point in coords if point is not None])

def lidar_lightring_task(scheduler: "TaskSchedular"):
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)
    if not companion.get_scans().ranges:
        return 
        
    robot.set_lights(companion.tools.lidar.get_motion_lightring(companion.get_scans().ranges))
