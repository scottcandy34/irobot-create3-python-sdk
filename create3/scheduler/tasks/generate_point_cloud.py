from torch import TYPE_CHECKING

from create3.models.common import Nodes
from create3 import CompanionNode

if TYPE_CHECKING:
    from create3.scheduler import TaskScheduler

def generate_point_cloud_task(scheduler: "TaskScheduler") -> None:
    """Generate and maintain a clean, filtered point cloud from the latest LiDAR coordinates.

    This task takes the raw output from GENERATE_COORDS and merges new points
    into an accumulated point cloud while removing duplicates using voxel-grid
    downsampling (fast and effective for real-time robotics).

    The resulting point cloud is stored in scheduler._outputs under
    Tasks.GENERATE_POINT_CLOUD for use by WALL_DETECTION, COLUMN_DETECTION,
    and any other perception tasks.
    """
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)
    coords: list[tuple[float, float]] | None = scheduler.get_task_output(companion.tasks.GENERATE_COORDS)
    if coords is None:
        return
    
    # Get the current accumulated point cloud (or start fresh)
    current_cloud: list[tuple[float, float]] = scheduler.get_task_output(companion.tasks.GENERATE_POINT_CLOUD)

    # Merge and filter using voxel downsampling (fast + uniform spacing)
    updated_cloud = companion.tools.point_cloud.merge_and_filter_voxel(current_cloud=current_cloud, new_points=coords, voxel_size_cm=2.0)

    # Store the updated cloud for other tasks
    scheduler.set_task_output(companion.tasks.GENERATE_POINT_CLOUD, updated_cloud)