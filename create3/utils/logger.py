
from colorama import init, Fore, Style

from rclpy.node import Node

from .other import object_to_string

init(autoreset=True)

class Logger():
    """A class to provide logging capabilities to nodes and other classes."""

    node: Node

    def print(self, msg):
        """Prints a value to node Info stream"""
        self.node.get_logger().info(object_to_string(msg))

    def print_notice(self, msg: str):
        """Prints a value to node Info stream as Cyan"""
        self.node.get_logger().info(Fore.CYAN + object_to_string(msg))

    def print_healthy(self, msg):
        """Prints a value to node Info stream as Green"""
        self.node.get_logger().info(Fore.GREEN + object_to_string(msg))

    def print_fatal(self, msg):
        """Prints a value to node Fatal stream"""
        self.node.get_logger().fatal(object_to_string(msg))

    def print_error(self, msg):
        """Prints a value to node Error stream"""
        self.node.get_logger().error(object_to_string(msg))

    def print_warning(self, msg):
        """Prints a value to node Warning stream"""
        self.node.get_logger().warning(object_to_string(msg))