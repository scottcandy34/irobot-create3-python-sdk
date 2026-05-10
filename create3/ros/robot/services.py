#
# Service Interface for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from rclpy.client import Client
from irobot_create_msgs.srv import ResetPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from create3.utils import Logger
from create3.models.robot import Services
from create3.utils.common.other import TIMEOUT, DEFAULT_WAIT


class ServiceClient(Logger):
    """ROS service client manager for the iRobot Create3.

    Provides a clean interface to call robot services (currently only
    `reset_pose`). All service calls are placed in a mutually exclusive
    callback group so they never interfere with subscriptions or other callbacks.

    The class also registers itself with the watchdog for interface monitoring.
    """

    def __init__(self, *args, **kwargs) -> None:
        """Initialize the service client and wait for the required services.

        Parameters
        ----------
        node : Node
            The ROS node that owns this service client.
        """
        super().__init__(*args, **kwargs)

        # Use a mutually exclusive callback group so service calls never block
        # other callbacks (subscriptions, timers, etc.)
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Register with watchdog for interface monitoring
        self.clients: list[Client] = []
        
    def find(self, name: Services) -> Client:
        for service in self.clients:
            if name == service.srv_name:
                return service
            
        return None
        
    def send_reset_navigation(self, reset_pose_request: ResetPose.Request) -> None:
        if not self.find(Services.RESET_POSE):
            client = self.node.create_client(ResetPose, Services.RESET_POSE, callback_group=self.callback_group)
            client.wait_for_service(timeout_sec=TIMEOUT)
            self.clients.append(client)
            
        self.find(Services.RESET_POSE).call(reset_pose_request, DEFAULT_WAIT)

        