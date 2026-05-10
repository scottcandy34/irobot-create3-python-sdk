#
# Detections Visualizer Example for iRobot Create3
# =====================================================================
# Created by scottcandy34 • Revised April 2026
#
# Simple, clean example that visualizes only wall detections in real time.
#
# This example demonstrates:
#   • Starting the minimal required tasks (HISTORY_KEEPER + WALL_DETECTION)
#   • Using the DetectionsVisualizer with only wall data
#   • Clean shutdown on window close
#
# The visualizer now focuses exclusively on wall segments and robot pose.
# =====================================================================

import sys
from pathlib import Path

# Make example runnable from both root and examples/
script_dir = Path(__file__).parent.resolve()
if (script_dir / "create3").exists():
    sys.path.insert(0, str(script_dir))
elif (script_dir.parent / "create3").exists():
    sys.path.insert(0, str(script_dir.parent))

from create3 import RobotNode, CompanionNode
from create3.models.common import Tasks
from create3.utils.display import DetectionsVisualizer


def main() -> None:
    """Run the live wall detections visualizer example."""

    robot = RobotNode(enable_scheduler=True)
    companion = CompanionNode(enable_scheduler=True)

    # Start only the tasks needed for wall detection
    robot.scheduler.add_task(Tasks.HISTORY_KEEPER)
    robot.scheduler.add_task(companion.tasks.WALL_DETECTION)

    # Launch visualizer (only walls + robot pose)
    DetectionsVisualizer(
        robot=robot,
        get_walls=lambda: robot.scheduler.get_task_output(companion.tasks.WALL_DETECTION),
    )

    # Clean shutdown when visualizer window is closed
    robot.shutdown()
    companion.shutdown()


if __name__ == "__main__":
    main()