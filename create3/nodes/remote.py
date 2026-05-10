#
# Remote Node for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from create3.models.common import Nodes
from create3.models.remote import Tasks
from create3.utils import remote as tools
from create3.utils.common.other import TIMEOUT
from create3.utils import rclpy, Watchdog, get_watchdog
from create3.scheduler import TaskScheduler, get_task_scheduler
from create3.ros.remote import InterfaceMixin

class RemoteNode(InterfaceMixin):
    """Main remote control node for the iRobot Create3.

    This node typically runs on a laptop or computer and provides:
      • Joystick input (controller)
      • Map and YOLO detections (if available)
      • Publishing capabilities for commands (rumble, etc.)

    It combines Publisher, Subscriber, and Threading via multiple inheritance.
    """

    def __init__(self, enable_watchdog: bool = True, enable_scheduler: bool = False) -> None:
        """Create and start the remote node.

        Parameters
        ----------
        enable_watchdog : bool
            Whether to register this node with the global watchdog.
        """
        # Safe ROS initialization (only once)
        rclpy.init()
        node = rclpy.create_node(Nodes.CREATE3_REMOTE)

        # Initialize the multiple-inheritance chain:
        # Publisher → Subscriber → Threading
        super().__init__(node, "Computer")  # calls Publisher then Subscriber
        node.wait_for_node(Nodes.CREATE3_ROBOT, TIMEOUT)

        self.tools = tools
        """Tools and utilities available to the remote node."""

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

    def shutdown(self) -> None:
        """Gracefully shut down the remote node.

        Stops the watchdog watch, shuts down all ROS resources, and cleans up.
        """
        if self.watchdog:
            self.watchdog.stop(self)          # stop watchdog monitoring
        if self.scheduler:
            self.scheduler.stop(self)
        super().shutdown()                  # calls Threading.shutdown() + Publisher/Subscriber cleanup
        rclpy.shutdown()