#
# Nodes for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from .utils import rclpy, Threading, Debugger, robot, companion, remote
from .interfaces import robot, companion, remote

debugger = Debugger()

class RobotNode(robot.ActionClientInterface, robot.ServiceInterface, robot.PublisherInterface, robot.SubscriptionInterface, Threading):
    """Setup Robot node with multithreading, subscriptions, publishers, services and actions."""

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

class CompanionNode(companion.PublisherInterface, companion.SubscriptionInterface, Threading):
    """Setup Companion node with multithreading, subscriptions, publishers."""

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

class RemoteNode(remote.PublisherInterface, remote.SubscriptionInterface, Threading):
    """Setup Remote node with multithreading, subscriptions, publishers."""

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