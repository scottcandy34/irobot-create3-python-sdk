#
# Nodes for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from .utils import rclpy, Threading, Debugger, robot, companion, remote
from .ros import robot, companion, remote

debugger = Debugger()

class RobotNode(robot.ActionClient, robot.ServiceClient, robot.Publisher, robot.Subscriber, Threading):
    """Setup Robot node with multithreading, subscribers, publishers, services and actions."""

    def __init__(self, useGoal = True):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('create3_ros_examples')

        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Create3"
        self._useGoal = useGoal

        self.tools = robot

        # Start the Threading/Spinning
        self.start()
        
        # Add node to Debugger
        debugger.add_device(self)

        # Reset the robot position to 0, 0, 0
        self.reset_navigation()

    def shutdown(self):
        debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()

class CompanionNode(companion.Publisher, companion.Subscriber, Threading):
    """Setup Companion node with multithreading, subscribers, publishers."""

    def __init__(self):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('companion_ros_examples')

        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Companion"

        self.tools = companion

        # Start the Threading/Spinning
        self.start()

        # Add node to Debugger
        debugger.add_device(self)

    def shutdown(self):
        debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()

class RemoteNode(remote.Publisher, remote.Subscriber, Threading):
    """Setup Remote node with multithreading, subscribers, publishers."""

    def __init__(self):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('remote_ros_examples')

        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Remote"

        self.tools = remote

        # Start the Threading/Spinning
        self.start()

        # Add node to Debugger
        debugger.add_device(self)

    def shutdown(self):
        debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()