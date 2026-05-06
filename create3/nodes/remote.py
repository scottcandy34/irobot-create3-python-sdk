#
# Remote Node for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from create3.models.common import Nodes
from create3.models.remote import Tasks
from create3.utils import remote as tools
from create3.utils.common.other import TIMEOUT
from create3.utils import rclpy, Threading, Debugger, get_debugger
from create3.ros.remote import Interface

class RemoteNode(Interface, Threading):
    """Main remote control node for the iRobot Create3.

    This node typically runs on a laptop or computer and provides:
      • Joystick input (controller)
      • Map and YOLO detections (if available)
      • Publishing capabilities for commands (rumble, etc.)

    It combines Publisher, Subscriber, and Threading via multiple inheritance.
    """

    def __init__(self, enable_debugger: bool = True) -> None:
        """Create and start the remote node.

        Parameters
        ----------
        enable_debugger : bool
            Whether to register this node with the global debugger.
        """
        # Safe ROS initialization (only once)
        rclpy.init()
        node = rclpy.create_node(Nodes.CREATE3_REMOTE)
        node._logger.name = "Computer"

        # Initialize the multiple-inheritance chain:
        # Publisher → Subscriber → Threading
        super().__init__(node)  # calls Publisher then Subscriber
        node.wait_for_node(Nodes.CREATE3_ROBOT, TIMEOUT)

        self.tools = tools
        """Tools and utilities available to the remote node."""

        self.tasks = Tasks
        """Available tasks that can be added to the TaskSchedular."""

        # Start background ROS spinning
        self.start()

        # Register with global debugger (optional)
        self.debugger: Debugger = None
        if enable_debugger:
            self.debugger = get_debugger()
            self.debugger.add_device(self)

    def shutdown(self) -> None:
        """Gracefully shut down the remote node.

        Stops the debugger watch, shuts down all ROS resources, and cleans up.
        """
        if self.debugger:
            self.debugger.stop(self)          # stop debugger monitoring
        super().shutdown()                  # calls Threading.shutdown() + Publisher/Subscriber cleanup
        rclpy.shutdown()