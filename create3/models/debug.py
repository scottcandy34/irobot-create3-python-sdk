#
# Debug Information for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

class Debug:
    """Container holding ROS interface metadata for a single device/node.

    Used by the global `Debugger` to monitor subscriptions, publishers,
    actions, and services. Also stores uptime/frequency statistics for
    each topic.
    """

    subscriptions: list[Subscription] = []
    publishers: list[Publisher] = []
    actions: list[ActionClient] = []
    services: list[Client] = []
    uptime: dict[str, list[int]] = {}

    def is_alive(self) -> list[tuple[str, bool]]:
        """Return a list of all ROS interfaces belonging to this device.

        Format: list of `(interface_name, True)` tuples.
        Used by the Debugger to track which interfaces are present.
        """
        subs = [(sub.topic_name, True) for sub in self.subscriptions]
        pubs = [(pub.topic_name, True) for pub in self.publishers]
        acts = [(act._action_name, True) for act in self.actions]
        servs = [(srv.service_name, True) for srv in self.services]

        return subs + pubs + acts + servs