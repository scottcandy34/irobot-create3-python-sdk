#
# Debug Information for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

class Debug():
    subscriptions: list[Subscription] = []
    publishers: list[Publisher] = []
    actions: list[ActionClient] = []
    services: list[Client] = []
    uptime: dict[str, list[int]] = {}

    def isAlive(self) -> list[tuple[str, bool]]:
        """Returns a list of all interfaces part of this device node."""
        subscriptions = [(x.topic_name, True) for x in self.subscriptions]
        publishers = [(x.topic_name, True) for x in self.publishers]
        actions = [(x._action_name, True) for x in self.actions]
        services = [(x.service_name, True) for x in self.services]
        return subscriptions + publishers + actions + services