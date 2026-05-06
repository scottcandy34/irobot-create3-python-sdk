from typing import TYPE_CHECKING

from create3.models.common import Nodes
from create3.models.common import Tasks
from create3.models.common import Stamped, Position
from create3.models.companion import Lidar

if TYPE_CHECKING:
    from create3.scheduler import TaskScheduler
    from create3 import RobotNode, CompanionNode

def generate_coords_task(scheduler: "TaskScheduler") -> None:
    """Generate a motion-compensated (deskewed) world-frame point cloud from the latest LiDAR scan.

    This task:
      • Retrieves the most recent LiDAR scan (as Stamped[Lidar])
      • Uses the pose history from HISTORY_KEEPER to interpolate the robot's position
        at the exact time each ray was measured
      • Transforms all points into world coordinates with correct motion compensation

    The resulting point cloud is stored under `companion.tasks.GENERATE_COORDS`
    for use by WALL_DETECTION, COLUMN_DETECTION, visualizers, etc.
    """
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)
    
    # Get latest stamped LiDAR data
    lidar_stamped: Stamped[Lidar] = companion.subscriber.lidar

    # Get pose history for deskewing (from HISTORY_KEEPER task)
    history_key = f"{Tasks.HISTORY_KEEPER}_{companion.get_name()}_{robot.subscriber.position.name}"
    pose_history: list[Stamped[Position]] = scheduler.get_task_output(history_key)

    # Perform motion-compensated deskewing
    deskewed_points = companion.tools.lidar.deskew_lidar_scan(lidar_stamped=lidar_stamped, pose_history=pose_history)

    # Store result for other tasks and visualizers
    scheduler.set_task_output(companion.tasks.GENERATE_COORDS, deskewed_points)