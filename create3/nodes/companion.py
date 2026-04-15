#
# Companion Node for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from create3.models import Nodes
from create3.utils import companion as tools
from create3.utils import rclpy, Threading, global_debugger
from create3.ros.companion import Publisher, Subscriber

class CompanionNode(Publisher, Subscriber, Threading):
    """Setup Companion node with multithreading, subscribers, publishers."""

    def __init__(self, enable_debugger = True):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node(Nodes.CREATE3_COMPANION.name.lower())

        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Raspberry"

        self.tools = tools

        # Start the Threading/Spinning
        self.start()

        # Add node to Debugger
        if enable_debugger:
            global_debugger.add_device(self)

    def shutdown(self):
        global_debugger.stop(self) # stops debugger watching node
        super().shutdown() # trigger original code before it gets overwritten
        rclpy.shutdown()