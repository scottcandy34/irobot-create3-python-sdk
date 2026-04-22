#
# Service Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from typing import TYPE_CHECKING

from irobot_create_msgs.srv import ResetPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.utils.common.other import TIMEOUT, DEFAULT_WAIT

class ServiceClient(Threading if TYPE_CHECKING else object):
    """Handle ROS Services by sending messages."""

    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        service_callback_group = MutuallyExclusiveCallbackGroup()

        # Create Service Clients
        self._reset_pose = self.node.create_client(ResetPose, 'reset_pose', callback_group=service_callback_group)
        self._reset_pose.wait_for_service(TIMEOUT)

        # Add services to debugger
        self.debug.services = [self._reset_pose]
        
    def reset_navigation(self):
        """Request that the robot resets position and heading."""
        # Reset Pose to 0,0 upon start of code
        self.print_warning("Resetting robot position. Max time 4sec.")
        self._reset_pose.call(ResetPose.Request(), DEFAULT_WAIT)
        time.sleep(1)