#
# RCLPY overrides for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import rclpy as rclpy_original
from rclpy.node import Node


# This is only to force rclpy to be initialized once. THIS IS NOT STANDARD PRACTICE FOR ROS
class rclpy:
    """A collection of functions for writing a ROS program."""
    _hasStarted = False
    _startedCount = 0

    @classmethod
    def init(cls):
        cls._startedCount +=1
        if not cls._hasStarted:
            rclpy_original.init()
            cls._hasStarted = True

    @classmethod
    def shutdown(cls):
        cls._startedCount -=1
        if cls._hasStarted and cls._startedCount == 0:
            rclpy_original.shutdown()
            cls._hasStarted = False
    
    def create_node(node_name: str) -> Node:
        return rclpy_original.create_node(node_name)