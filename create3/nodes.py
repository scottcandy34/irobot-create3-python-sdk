#
# ROS Node Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from .rclpy import rclpy
from .interfaces.actions import RobotActionClients
from .interfaces.services import RobotServices
from .interfaces.publishers import RobotPublishers
from .interfaces.subscriptions import RobotSubscriptions
from .threading import RobotThreading

class RobotNode(RobotActionClients, RobotServices, RobotPublishers, RobotSubscriptions, RobotThreading):
    """Setup Robot node with multithreading, subscriptions, publishers, services and actions."""
    def __init__(self, useGoal = True):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('create3_ros_examples')

        super().__init__(node) # trigger original code before it gets overwritten
        self._useGoal = useGoal

        # Start the Threading/Spinning
        self.start()
        
        # Reset the robot position to 0, 0, 0
        self.reset_navigation()

    def shutdown(self):
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()