#
# Companion Node for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from create3.models.common import Nodes
from create3.models.companion import Tasks
from create3.utils import companion as tools
from create3.utils.common.other import TIMEOUT
from create3.utils import rclpy, Threading, Watchdog, get_watchdog
from create3.scheduler import TaskScheduler, get_task_scheduler
from create3.ros.companion import Interface

class CompanionNode(Interface, Threading):
    """Main companion node for the iRobot Create3.

    Combines:
      • Publisher (servo control)
      • Subscriber (LiDAR + ultrasonic)
      • Threading (background ROS spinning + logging)

    This is the central node that runs on the companion computer (Raspberry Pi).
    It automatically starts spinning, registers itself with the global watchdog,
    and exposes `tools` and `tasks` for easy access from other parts of the SDK.
    """

    def __init__(self, enable_watchdog: bool = True, enable_scheduler: bool = False) -> None:
        """Create and start the companion node.

        Parameters
        ----------
        enable_watchdog : bool
            Whether to register this node with the global watchdog.
        """
        # Safe ROS initialization (only once)
        rclpy.init()
        node = rclpy.create_node(Nodes.CREATE3_COMPANION)
        node._logger.name = "Raspberry"

        # Initialize the multiple-inheritance chain:
        # Publisher → Subscriber → Threading
        super().__init__(node)
        node.wait_for_node(Nodes.CREATE3_ROBOT, TIMEOUT)

        self.tools = tools
        """Tools and utilities for working with LiDAR, perception, etc."""

        self.tasks = Tasks
        """Available tasks that can be added to the TaskSchedular."""

        # Start background ROS spinning
        self.start()

        # Register with global watchdog (optional)
        self.watchdog: Watchdog = None
        if enable_watchdog:
            self.watchdog = get_watchdog()
            self.watchdog.add_device(self)
            
        self.scheduler: TaskScheduler = None
        if enable_scheduler:
            self.scheduler = get_task_scheduler()
            self.scheduler.add_device(self)
            
        # Move servo to default position on startup
        self.reset_servo()

    def shutdown(self) -> None:
        """Gracefully shut down the companion node.

        Stops the watchdog watch, shuts down all ROS resources, and cleans up.
        """
        if self.watchdog:
            self.watchdog.stop(self)          # stop watchdog monitoring
        if self.scheduler:
            self.scheduler.stop(self)
        super().shutdown()                  # calls Threading.shutdown() + Publisher/Subscriber cleanup
        rclpy.shutdown()