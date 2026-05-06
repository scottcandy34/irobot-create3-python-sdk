from typing import Any, Callable, Type

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

from create3.models.common import SubscriptionStats

class MonitoredSubscription:
    """Thin wrapper that adds stats tracking to any subscription."""

    def __init__(
        self,
        node: Node,
        msg_type: Type[Any],
        topic: str,
        callback: Callable[[Any], None],
        qos_profile: QoSProfile,
        callback_group=None,
        **kwargs,
    ):
        self.stats = SubscriptionStats(topic=topic)

        def wrapped_callback(msg: Any):
            current_ns = node.get_clock().now().nanoseconds
            self.stats.update(current_ns)
            callback(msg)

        self._subscription: Subscription = node.create_subscription(
            msg_type=msg_type,
            topic=topic,
            callback=wrapped_callback,
            qos_profile=qos_profile,
            callback_group=callback_group,
            **kwargs,
        )

    # Forward all real subscription attributes/methods
    def __getattr__(self, name: str):
        return getattr(self._subscription, name)

    @property
    def topic_name(self):
        return self._subscription.topic_name