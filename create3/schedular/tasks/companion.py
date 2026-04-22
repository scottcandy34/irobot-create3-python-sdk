from torch import TYPE_CHECKING

from geometry_msgs.msg import Twist
    
from create3.models import Nodes
from create3.models.companion import Tasks
from create3 import RobotNode, CompanionNode, RemoteNode

if TYPE_CHECKING:
    from create3.schedular import TaskSchedular

def generate_coords_task(scheduler: "TaskSchedular") -> None:
    """Generate world-frame (x, y) coordinates from the latest LiDAR scan.

    Uses the companion node's LiDAR data and the robot's current pose
    to transform every ray into a 2D point cloud in world coordinates.
    """
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)

    lidar = companion._subscription_msgs.lidar

    scheduler._outputs[Tasks.GENERATE_COORDS] = [
        companion.tools.lidar.get_coords(lidar.ranges, index, robot.get_position())
        for index in range(lidar.size())
    ]

def wall_detection_task(scheduler: "TaskSchedular") -> None:
    """Run line-segment (wall) detection on the latest point cloud.

    Requires the GENERATE_COORDS task to have run first.
    """
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    coords: list[tuple[float, float]] | None = scheduler.get_task_output(Tasks.GENERATE_COORDS)
    if coords is None:
        return

    # Filter out invalid (None) points before detection
    valid_points = [p for p in coords if p is not None]

    scheduler._outputs[Tasks.WALL_DETECTION] = (companion.tools.perception.detectors.find_line_segments(valid_points))

def column_detection_task(scheduler: "TaskSchedular") -> None:
    """Run circular arc (column/obstacle) detection on the latest point cloud.

    Requires the GENERATE_COORDS task to have run first.
    """
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    coords: list[tuple[float, float]] | None = scheduler.get_task_output(Tasks.GENERATE_COORDS)
    if coords is None:
        return

    # Filter out invalid (None) points before detection
    valid_points = [p for p in coords if p is not None]

    scheduler._outputs[Tasks.COLUMN_DETECTION] = (companion.tools.perception.detectors.find_circle_arcs(valid_points))

def lidar_lightring_task(scheduler: "TaskSchedular") -> None:
    """Update the robot's lightring LEDs based on the closest LiDAR obstacle."""
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)

    if not companion.get_scans():
        return

    robot.set_lights(companion.tools.lidar.get_motion_lightring(companion.get_scans()))

def simple_wall_follower(scheduler: "TaskSchedular") -> None:
    """Simple reactive wall-follower using LiDAR and PID control.

    Sends velocity commands directly to the robot.
    """
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)

    if not companion.get_scans():
        return

    lidar = companion._subscription_msgs.lidar

    twist_msg = companion.tools.wall_follow.pid_lidar_to_twist(lidar)
    robot.send_twist(twist_msg)