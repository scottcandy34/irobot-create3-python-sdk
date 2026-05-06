#
# Logger for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import Any
from colorama import init, Fore, Style

from .node import Node
from .common.other import object_to_string

init(autoreset=True)

class Logger:
    """ROS node logger wrapper with colored output for different severity levels.

    Provides convenient methods to log messages at INFO, NOTICE, HEALTHY,
    ERROR, WARNING, and FATAL levels. All messages are passed through
    `object_to_string` for safe string conversion (handles non-string objects).

    Usage:
        logger = Logger(node)
        logger.print("Normal message")
        logger.print_healthy("System is running great!")
        logger.print_error("Something went wrong")
    """

    def __init__(self, node: Node) -> None:
        """Initialize the logger with a ROS node.

        Parameters
        ----------
        node : Node
            The rclpy Node instance whose logger will be used.
        """
        self.node = node

    def print(self, msg: Any) -> None:
        """Log a message at INFO level (white/default color)."""
        self.node.get_logger().info(object_to_string(msg))

    def print_notice(self, msg: Any) -> None:
        """Log a message at INFO level in cyan (for notices)."""
        self.node.get_logger().info(Fore.CYAN + object_to_string(msg))

    def print_healthy(self, msg: Any) -> None:
        """Log a message at INFO level in green (for healthy status)."""
        self.node.get_logger().info(Fore.GREEN + object_to_string(msg))

    def print_fatal(self, msg: Any) -> None:
        """Log a message at FATAL level (red + system shutdown behavior)."""
        self.node.get_logger().fatal(object_to_string(msg))

    def print_error(self, msg: Any) -> None:
        """Log a message at ERROR level (red)."""
        self.node.get_logger().error(object_to_string(msg))

    def print_warning(self, msg: Any) -> None:
        """Log a message at WARNING level (yellow)."""
        self.node.get_logger().warning(object_to_string(msg))