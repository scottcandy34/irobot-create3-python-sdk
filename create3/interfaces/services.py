#
# ROS Topic Publisher Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from typing import TYPE_CHECKING

from irobot_create_msgs.srv import ResetPose

from ..objects import TIMEOUT, DEFAULT_WAIT
from ..threading import RobotThreading

class RobotServices(RobotThreading if TYPE_CHECKING else object):
    """Handle ROS Services by sending messages."""
    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Create Service Clients
        self._reset_pose = self.node.create_client(ResetPose, 'reset_pose', callback_group=self._actionCbGroup)
        self._reset_pose.wait_for_service(TIMEOUT)

    def reset_navigation(self):
        """Request that the robot resets position and heading."""
        # Reset Pose to 0,0 upon start of code
        self.print_warning("Resetting robot position. Max time 4sec.")
        self._reset_pose.call(ResetPose.Request(), DEFAULT_WAIT)
        time.sleep(1)