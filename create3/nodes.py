#
# ROS Node Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from .utils import rclpy
from .debugger import RclpyDebugger
from .interfaces import RobotActionClients, RobotServices, RobotPublishers, RPIPublishers, PCPublishers, RobotSubscriptions, RPISubscriptions, PCSubscriptions
from .threading import RobotThreading, RPIThreading, PCThreading

debugger = RclpyDebugger()

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
        
        # Add node to Debugger
        debugger.add_device(self)

        # Reset the robot position to 0, 0, 0
        self.reset_navigation()

    def shutdown(self):
        debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()

class RpiNode(RPIPublishers, RPISubscriptions, RPIThreading):
    """Setup Rpi node with multithreading, subscriptions, publishers."""
    def __init__(self):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('rpi_ros_examples')

        super().__init__(node) # trigger original code before it gets overwritten

        # Start the Threading/Spinning
        self.start()

        # Add node to Debugger
        debugger.add_device(self)

    def shutdown(self):
        debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()

class PcNode(PCPublishers, PCSubscriptions, PCThreading):
    """Setup PC node with multithreading, subscriptions, publishers."""
    def __init__(self):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('pc_ros_examples')

        super().__init__(node) # trigger original code before it gets overwritten

        # Start the Threading/Spinning
        self.start()

        # Add node to Debugger
        debugger.add_device(self)

    def shutdown(self):
        debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()