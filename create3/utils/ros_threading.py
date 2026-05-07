#
# Threading for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import asyncio
import time
from threading import Thread

from rclpy.executors import SingleThreadedExecutor

from .logger import Logger
from .node import Node

class Threading(Logger):
    """ROS node threading helper with background spinning, timing utilities,
    topic uptime/frequency tracking, and delayed one-shot callbacks.

    Inherits from `Logger` so all colored logging methods (`print_healthy`,
    `print_error`, etc.) are available.
    """

    def __init__(self, node: Node) -> None:
        """Initialize threading support for a ROS node.

        Parameters
        ----------
        node : Node
            The rclpy node this helper will manage.
        """
        super().__init__(node)          # properly initialize parent Logger

    def time(self) -> int:
        """Return the current ROS clock time in nanoseconds."""
        return self.node.get_clock().now().nanoseconds

    def get_name(self) -> str:
        """Return the name of this ROS node (useful for logging)."""
        return self.node.get_name()

    def start(self) -> None:
        """Start ROS spinning in a background thread.

        This allows the main thread to continue while the node processes
        topics, services, and actions.
        """
        self.print(f"{self.get_name()} node is initiating... "
                   "Listening for Topics, Services and Actions.")

        self._executor = SingleThreadedExecutor()
        self._ros_thread = Thread(target=self._spin, daemon=True)
        self._ros_thread.start()

    def shutdown(self) -> None:
        """Gracefully stop the ROS spinning thread and destroy the node."""
        self._executor.shutdown()

        # Wait for the thread to finish cleanly
        while self._ros_thread.is_alive():
            time.sleep(0.1)
        self._ros_thread.join()

        self.print_warning(f"{self.get_name()} node has shutdown.")
        self.node.destroy_node()
        
    def _spin(self) -> None:
        """Internal method that runs the ROS executor in a background thread.

        Do not call this directly — use `start()` instead.
        """
        self._executor.add_node(self.node)
        self._executor.spin()
        
    async def _run_sync_method(self, method, *args, **kwargs):
        """Internal helper: safely runs any synchronous method in the async loop."""
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(
            None, lambda: method(*args, **kwargs)
        )