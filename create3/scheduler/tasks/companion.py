from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
    
from create3.models.common import Nodes
from create3.models.common import Tasks
from create3 import RobotNode, CompanionNode, RemoteNode
from create3.models.common import Stamped, Position
from create3.models.companion import Lidar

if TYPE_CHECKING:
    from create3.scheduler import TaskScheduler

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
    
def wall_detection_task(scheduler: "TaskScheduler") -> None:
    """Run line-segment (wall) detection on the latest point cloud.

    Requires the GENERATE_COORDS task to have run first.
    """
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    coords: list[tuple[float, float]] | None = scheduler.get_task_output(companion.tasks.GENERATE_COORDS)
    if coords is None:
        return

    # Filter out invalid (None) points before detection
    valid_points = [p for p in coords if p is not None]

    scheduler.set_task_output(companion.tasks.WALL_DETECTION, companion.tools.perception.detectors.find_line_segments(valid_points))

def lidar_lightring_task(scheduler: "TaskScheduler") -> None:
    """Update the robot's lightring LEDs based on the closest LiDAR obstacle."""
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)

    if not companion.get_scans():
        return

    robot.set_lights(companion.tools.lidar.get_motion_lightring(companion.get_scans()))

def simple_wall_follower(scheduler: "TaskScheduler") -> None:
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
