#
# Robot Node for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from create3.utils import robot as tools
from create3.utils import rclpy, Threading, global_debugger
from create3.ros.robot import ActionClient, ServiceClient, Publisher, Subscriber

class RobotNode(ActionClient, ServiceClient, Publisher, Subscriber, Threading):
    """Setup Robot node with multithreading, subscribers, publishers, services and actions."""

    def __init__(self, useGoal = True):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('create3_robot')

        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Create3"
        self._useGoal = useGoal

        self.tools = tools

        # Start the Threading/Spinning
        self.start()
        
        # Add node to Debugger
        global_debugger.add_device(self)

        # Reset the robot position to 0, 0, 0
        self.reset_navigation()

    def shutdown(self):
        global_debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()