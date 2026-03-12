#
# Remote Node for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from create3.utils import remote as tools
from create3.utils import rclpy, Threading, global_interrupt, global_debugger
from create3.ros.remote import Publisher, Subscriber

class RemoteNode(Publisher, Subscriber, Threading):
    """Setup Remote node with multithreading, subscribers, publishers."""

    def __init__(self, enable_debugger = True):
        # Initialize ROS2 node
        rclpy.init()
        node = rclpy.create_node('create3_remote')

        global_interrupt.add_device(self)

        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Remote"

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