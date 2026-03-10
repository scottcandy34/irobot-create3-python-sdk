#
# ROS Node Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from .utils import rclpy, Threading, Debugger, robot, rpi, pc
from .interfaces import robot, rpi, pc

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

class RpiNode(rpi.PublisherInterface, rpi.SubscriptionInterface, Threading):
    """Setup Rpi node with multithreading, subscriptions, publishers."""

    def __init__(self):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('rpi_ros_examples')

        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Raspberry Pi"

        self.tools = rpi

        # Start the Threading/Spinning
        self.start()

        # Add node to Debugger
        debugger.add_device(self)

    def shutdown(self):
        debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()

class PcNode(pc.PublisherInterface, pc.SubscriptionInterface, Threading):
    """Setup PC node with multithreading, subscriptions, publishers."""
    
    def __init__(self):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('pc_ros_examples')

        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Remote PC"

        self.tools = pc

        # Start the Threading/Spinning
        self.start()

        # Add node to Debugger
        debugger.add_device(self)

    def shutdown(self):
        debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()