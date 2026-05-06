#
# RCLPY overrides for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import rclpy as _rclpy

from .node import Node

class rclpy:
    """Singleton-style wrapper around rclpy to guarantee initialization and shutdown happen only once.

    This is a **non-standard** pattern created specifically for compatibility with
    the iRobot Create3 SDK. It prevents multiple calls to `rclpy.init()` (which
    would otherwise raise an error) when the SDK is imported/used in different
    contexts or across multiple modules.

    It uses a simple reference-counting mechanism:
      • `init()` is called only on the very first request.
      • `shutdown()` is called only when the last user/context releases it.
    """

    _has_started: bool = False
    _started_count: int = 0

    @classmethod
    def init(cls) -> None:
        """Initialize rclpy only if it has not already been initialized."""
        cls._started_count += 1
        if not cls._has_started:
            _rclpy.init()
            cls._has_started = True

    @classmethod
    def shutdown(cls) -> None:
        """Shutdown rclpy only when the last active context calls it.

        The count is clamped to zero to guard against accidental negative values.
        """
        cls._started_count -= 1
        if cls._has_started and cls._started_count <= 0:
            _rclpy.shutdown()
            cls._has_started = False
            cls._started_count = 0  # prevent negative count

    @classmethod
    def create_node(cls, node_name: str) -> Node:
        """Create and return a new ROS 2 node (delegates to the real rclpy)."""
        return Node(node_name)

    @classmethod
    def count_nodes(cls) -> int:
        """Return the current number of active node contexts tracked by this wrapper."""
        return cls._started_count