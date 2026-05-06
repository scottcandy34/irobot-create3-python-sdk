from typing import Any, Callable, Type

from rclpy.timer import Timer
from rclpy.qos import QoSProfile
from rclpy.node import Node as _node
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.publisher import Publisher

from .monitored_subscription import MonitoredSubscription

class Node(_node):
    def create_monitored_subscription(self, msg_type: Type[Any], topic: str, callback: Callable[[Any], None], qos_profile: QoSProfile, callback_group=None, **kwargs):
        return MonitoredSubscription(self, msg_type, topic, callback, qos_profile, callback_group, **kwargs)
        
    def create_oneshot_timer(self, delay_time: float | int, callback: Callable, *args: Any, **kwargs: Any) -> Timer:
        """Schedule a one-shot callback to run after a delay.

        The timer is automatically destroyed after it fires (so it runs only once).

        Parameters
        ----------
        delay_time : float | int
            Delay in seconds before the callback runs.
        callback : Callable
            Function to call after the delay.
        *args, **kwargs
            Arguments passed to the callback.

        Returns
        -------
        Timer
            The created timer object (in case you need to cancel it early).
        """
        timer: Timer | None = None

        def one_shot_wrapper() -> None:
            nonlocal timer
            # Destroy the timer so it never fires again
            if timer is not None:
                self.destroy_timer(timer)
                timer = None

            try:
                callback(*args, **kwargs)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"Delayed callback failed: {e}")

        # Create the timer (one-shot)
        timer = self.create_timer(delay_time, one_shot_wrapper)
        return timer
    
    def test_subscription(self, interface: MonitoredSubscription) -> bool:
        """Return True if at least one publisher exists for this topic."""
        pub_info = self.get_publishers_info_by_topic(interface.topic_name)
        return len(pub_info) > 0

    def test_publisher(self, interface: Publisher) -> bool:
        """Return True if at least one subscriber exists for this topic."""
        sub_info = self.get_subscriptions_info_by_topic(interface.topic_name)
        return len(sub_info) > 0

    def test_action_client(self, interface: ActionClient) -> bool:
        """Return True if the action server is ready/available."""
        return interface.server_is_ready()

    def test_service_client(self, interface: Client) -> bool:
        """Return True if the service server is ready/available."""
        return interface.service_is_ready()