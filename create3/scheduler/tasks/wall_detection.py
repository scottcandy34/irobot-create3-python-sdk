from typing import TYPE_CHECKING

from create3.models.common import Nodes

if TYPE_CHECKING:
    from create3 import CompanionNode
    from create3.scheduler import TaskScheduler

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
