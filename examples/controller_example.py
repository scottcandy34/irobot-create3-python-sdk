#
# Controller Visualizer Example for iRobot Create3
# ================================================
#
# This example demonstrates real-time visualization of a PlayStation-style
# controller connected to the remote node.
#
# Features:
#   • Live controller visualizer (joysticks, triggers, buttons, D-pad)
#   • Button callback system (Options button toggles docking/undocking)
#   • Starts the CONTROLLER task to read joystick input
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

from create3 import RobotNode, RemoteNode
from create3.schedular import TaskSchedular
from create3.utils.display import ControllerVisualizer

def main() -> None:
    """Run the Controller Visualizer example with docking button support."""

    # Initialize nodes
    robot = RobotNode()
    remote = RemoteNode()

    # Create task scheduler and register devices
    task_schedular = TaskSchedular()
    task_schedular.add_device(robot)
    task_schedular.add_device(remote)

    # Start the controller input task
    task_schedular.add_task(remote.tasks.CONTROLLER)

    # Register docking/undocking callback on the Options button (rising edge)
    @remote.get_controller().buttons.options.pressed
    def docking():
        if robot.get_docking_values().is_docked:
            robot.print("Undocking")
            robot.undock()
            robot.print("Undocking Completed")
        else:
            robot.print("Docking")
            robot.dock()
            robot.print("Docking Completed")

    # Launch the live controller visualizer
    ControllerVisualizer(remote)

    # Clean shutdown
    task_schedular.shutdown()
    robot.shutdown()
    remote.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()