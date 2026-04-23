#
# PID Tuner Example for iRobot Create3
# ====================================
#
# This example launches the robot and companion nodes, starts a simple
# wall-follower task, and opens the live PID Tuner GUI.
#
# You can adjust Kp, Ki, and Kd in real time while the robot drives.
# The GUI updates every 50 ms showing error, P/I/D terms, and output.
#
# Controls (in the tuner window):
#   1 / 2 / 3     → Select Kp / Ki / Kd
#   ↑ / ↓         → Increase / decrease selected gain
#   Tab           → Cycle step size (finer ↔ coarser)
#   Enter         → Type an exact value
#   Esc           → Close tuner and print final tuned gains
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
    # Running from project root
    sys.path.insert(0, str(script_dir))
elif (script_dir.parent / "create3").exists():
    # Running from inside examples/
    sys.path.insert(0, str(script_dir.parent))
# =============================================================================

from create3.utils.display import PIDTuner
from create3 import RobotNode, CompanionNode
from create3.schedular import TaskSchedular

def main() -> None:
    """Run the PID tuner example with simple wall following."""

    # Initialize nodes
    robot = RobotNode()
    companion = CompanionNode()

    # Create task scheduler and register devices
    task_schedular = TaskSchedular()
    task_schedular.add_device(robot)
    task_schedular.add_device(companion)

    # Start the wall follower task
    task_schedular.add_task(companion.tasks.SIMPLE_WALL_FOLLOWER)

    # Launch live PID tuner (tunes the angular PID used by wall following)
    PIDTuner(companion.tools.wall_follow.pid_angular)

    # Clean shutdown
    task_schedular.shutdown()
    robot.shutdown()
    companion.shutdown()

# =============================================================================
if __name__ == "__main__":
    main()