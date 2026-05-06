#
# Robot Node for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from create3.models.common import Nodes
from create3.models.robot import Tasks
from create3.utils import robot as tools
from create3.utils.common.other import TIMEOUT
from create3.utils import rclpy, Threading, Debugger, get_debugger
from create3.scheduler import TaskScheduler, get_task_scheduler
from create3.ros.robot import Interface

class RobotNode(Interface, Threading):
    """Main robot node for the iRobot Create3.

    Combines everything needed to control the physical robot:
      • ActionClient     (navigation, docking, driving, turning, LED animations, audio)
      • ServiceClient    (reset pose, etc.)
      • Publisher        (wheel speeds, lightring, audio)
      • Subscriber       (odometry, IR, hazards, battery, IMU, docking, etc.)
      • Threading        (background ROS spinning + logging)

    This is the central node that directly interfaces with the Create3 robot.
    """

    def __init__(self, enable_debugger: bool = True, enable_scheduler: bool = False) -> None:
        """Create and start the main robot node.

        Parameters
        ----------
        enable_debugger : bool
            Whether to register this node with the global debugger.
        use_goal : bool
            Whether to use high-level action goals (True) or low-level
            timed twist commands (False) for movement.
        """
        # Safe ROS initialization (only once)
        rclpy.init()
        node = rclpy.create_node(Nodes.CREATE3_ROBOT)
        node._logger.name = "Create3"

        # Initialize the multiple-inheritance chain in the correct MRO order
        super().__init__(node)  # ActionClient → ServiceClient → Publisher → Subscriber → Threading
        node.wait_for_node(Nodes.CREATE3_ROBOT, TIMEOUT)

        self.tools = tools
        """Tools and utilities for working with the robot's sensors and controls."""

        self.tasks = Tasks
        """Available tasks that can be added to the TaskSchedular."""

        # Start background ROS spinning
        self.start()
        
        # Register with global debugger (optional)
        self.debugger: Debugger = None
        if enable_debugger:
            self.debugger = get_debugger()
            self.debugger.add_device(self)
        
        self.scheduler: TaskScheduler = None
        if enable_scheduler:
            self.scheduler = get_task_scheduler()
            self.scheduler.add_device(self)

        # Reset the robot's position and heading to (0, 0, 0°) on startup
        self.reset_navigation()

    def shutdown(self) -> None:
        """Gracefully shut down the robot node.

        Stops the debugger watch, shuts down all ROS resources, and cleans up.
        """
        if self.debugger:
            self.debugger.stop(self)          # stop debugger monitoring
        if self.scheduler:
            self.scheduler.stop(self)
        super().shutdown()                  # calls Threading.shutdown() + all parent cleanup
        rclpy.shutdown()
