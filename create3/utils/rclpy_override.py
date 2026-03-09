#
# RCLPY overrides for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import rclpy as _rclpy
from rclpy.node import Node

# This is only to force rclpy to be initialized once. THIS IS NOT STANDARD PRACTICE FOR ROS
class rclpy:
    """
    Overrides for rclpy to ensure that it is only initialized once, and to provide a create_node function 
    that can be used to create a ROS node. This is not standard practice for ROS, but it is necessary to ensure 
    that the iRobot Create3 SDK can be used in a way that is consistent with the rest of the SDK, and to prevent 
    issues with multiple initializations of rclpy when using the SDK in different contexts (e.g. on a Raspberry Pi or on a PC).
    """
    _hasStarted = False
    _startedCount = 0

    @classmethod
    def init(cls):
        """Initialize rclpy if it hasn't been initialized yet."""
        cls._startedCount +=1
        if not cls._hasStarted:
            _rclpy.init()
            cls._hasStarted = True

    @classmethod
    def shutdown(cls):
        """Shutdown rclpy if it has been initialized and there are no more nodes using it."""
        cls._startedCount -=1
        if cls._hasStarted and cls._startedCount == 0:
            _rclpy.shutdown()
            cls._hasStarted = False
    
    @classmethod
    def create_node(cls, node_name: str) -> Node:
        """Create a ROS node with the given name."""
        return _rclpy.create_node(node_name)