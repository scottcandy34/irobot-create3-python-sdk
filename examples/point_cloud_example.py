#
# Live Point Cloud Example for iRobot Create3
# ===========================================
#
# Demonstrates the full point cloud pipeline with live visualization.
#
# Created by Scottcandy34
#

import sys
from pathlib import Path

# =============================================================================
# Make the example runnable from both examples/ and the project root
# =============================================================================
script_dir = Path(__file__).parent.resolve()

if (script_dir / "create3").exists():
    sys.path.insert(0, str(script_dir))
elif (script_dir.parent / "create3").exists():
    sys.path.insert(0, str(script_dir.parent))
# =============================================================================

from create3.models.common import Tasks
from create3 import RobotNode, RemoteNode, CompanionNode
from create3.scheduler import TaskScheduler
from create3.utils.display import PointCloudVisualizer

def main() -> None:
    """Run the live filtered point cloud visualization example."""

    robot = RobotNode()
    remote = RemoteNode()
    companion = CompanionNode()

    task_scheduler = TaskScheduler()
    task_scheduler.add_device(robot)
    task_scheduler.add_device(remote)
    task_scheduler.add_device(companion)

    task_scheduler.add_task(Tasks.HISTORY_KEEPER)
    task_scheduler.add_task(remote.tasks.CONTROLLER)
    task_scheduler.add_task(companion.tasks.GENERATE_COORDS)
    task_scheduler.add_task(companion.tasks.GENERATE_POINT_CLOUD)
    
    # Pass a lambda that directly returns the latest point cloud
    PointCloudVisualizer(lambda: task_scheduler.get_task_output(companion.tasks.GENERATE_POINT_CLOUD), robot)

    task_scheduler.shutdown()
    robot.shutdown()
    companion.shutdown()


if __name__ == "__main__":
    main()
    