#
# Service Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from typing import TYPE_CHECKING

from rclpy.node import Node
from irobot_create_msgs.srv import ResetPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Threading
from create3.utils.common.other import TIMEOUT, DEFAULT_WAIT


class ServiceClient(Threading if TYPE_CHECKING else object):
    """ROS service client manager for the iRobot Create3.

    Provides a clean interface to call robot services (currently only
    `reset_pose`). All service calls are placed in a mutually exclusive
    callback group so they never interfere with subscriptions or other callbacks.

    The class also registers itself with the debugger for interface monitoring.
    """

    def __init__(self, node: Node) -> None:
        """Initialize the service client and wait for the required services.

        Parameters
        ----------
        node : Node
            The ROS node that owns this service client.
        """
        super().__init__(node)  # initialize Threading + Logger

        # Use a mutually exclusive callback group so service calls never block
        # other callbacks (subscriptions, timers, etc.)
        service_callback_group = MutuallyExclusiveCallbackGroup()

        # Create service clients
        self._reset_pose = self.node.create_client(ResetPose, "reset_pose", callback_group=service_callback_group)

        # Wait for the service to become available
        self._reset_pose.wait_for_service(timeout_sec=TIMEOUT)

        # Register with debugger for interface monitoring
        self.debug.services = [self._reset_pose]

    def reset_navigation(self) -> None:
        """Request the robot to reset its position and heading to (0, 0, 0°).

        This is typically called once at startup. The robot takes up to ~4 seconds
        to complete the reset.
        """
        self.print_warning("Resetting robot position. Max time 4 sec.")

        # Send the reset request
        self._reset_pose.call(ResetPose.Request(), DEFAULT_WAIT)

        # Give the robot time to process the reset
        time.sleep(1.0)