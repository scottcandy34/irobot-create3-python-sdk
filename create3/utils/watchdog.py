#
# Watchdog for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread
from typing import Any
import colorama

from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.publisher import Publisher

from . import rclpy
from create3.models.common import Nodes
from .ros_threading import Threading
from .logger import Logger
from .monitored_subscription import MonitoredSubscription

UPTIME_FREQUENCY = 100 # in Hz
DEBUGGER_INTERVAL = 2 # in Hz

colorama.init(autoreset=True)

class Watchdog(Logger):
    """Background ROS interface watchdog and uptime monitor.

    Monitors every attached `Threading` device for:
      • Missing publishers/subscribers, action servers, or service servers
      • Topics publishing faster than `UPTIME_FREQUENCY` Hz (possible infinite loop)

    Runs in its own background thread and uses colored logging (inherited from `Logger`).
    """

    def __init__(self) -> None:
        """Start the watchdog node and its background watcher thread.

        Uses the custom `rclpy` to safely initialize only once.
        """
        # Create our own watchdog node (uses the custom rclpy)
        rclpy.init()
        node = rclpy.create_node(Nodes.ROS_WATCHDOG)

        # Initialize Logger parent with our node
        super().__init__(node, "Watchdog")

        self.print(f"{node.get_name()} node is initiating... Watching Topics, Services and Actions.")
        
        self._running = True
        self._devices: list[Threading] = []
        self._validated: dict[str, bool] = {}
        self._logged: dict[str, list[int]] = {}   # topic_name → list of recent timestamps (ns)

        self._thread = Thread(target=self._watcher, daemon=True)
        self._thread.start()

    def add_device(self, device: Threading) -> None:
        """Start watching a device's ROS interfaces and uptime statistics."""
        self._devices.append(device)
        self._validated.update(device.is_alive())  # copy initial validation state

    def remove_device(self, device: Threading) -> None:
        """Stop watching a device (removes it from the watchdog)."""
        for idx, obj in enumerate(self._devices):
            if obj.get_name() == device.get_name():
                self._devices.pop(idx)
                break

    def _check_interface(self, interface: Any) -> None:
        """Check one ROS interface and log healthy/error state changes."""

        if isinstance(interface, MonitoredSubscription):
            exist = self.node.test_subscription(interface)
            name = interface.topic_name
            type_ = "Topic Publisher"

        elif isinstance(interface, Publisher):
            exist = self.node.test_publisher(interface)
            name = interface.topic_name
            type_ = "Topic Subscriber"

        elif isinstance(interface, ActionClient):
            exist = self.node.test_action_client(interface)
            name = interface._action_name
            type_ = "Action Server"

        elif isinstance(interface, Client):
            exist = self.node.test_service_client(interface)
            name = interface.service_name
            type_ = "Service Server"

        else:
            raise ValueError(f"ROS interface type not recognized: {type(interface)}")

        # State change → log once
        previously_valid = self._validated.get(name, True)

        if not exist and previously_valid:
            self._validated[name] = False
            self.print_error(f"{type_} '{name}' is not available.")

        elif exist and not previously_valid:
            self._validated[name] = True
            self.print_healthy(f"{type_} '{name}' is now available.")

    def _watcher(self) -> None:
        """Main background loop that monitors all attached devices."""
        # Wait until at least one device is attached
        while not self._devices:
            time.sleep(0.1)

        while self._devices and self._running:
            for device in self._devices:
                # === Subscriptions (check publisher + frequency) ===
                if device.subscriber:
                    sub: MonitoredSubscription
                    for sub in device.subscriber.topics:
                        self._check_interface(sub)

                        freq = sub.stats.current_hz  # current frequency (Hz)
                        if freq >= UPTIME_FREQUENCY:
                            self._log_high_frequency_warning(sub.topic_name)

                # === Publishers, Actions, Services ===
                if device.publisher:
                    for pub in device.publisher.topics:
                        self._check_interface(pub)
                if device.actions:
                    for action in device.actions.clients:
                        self._check_interface(action)
                if device.services:
                    for service in device.services.clients:
                        self._check_interface(service)

            time.sleep(1.0 / DEBUGGER_INTERVAL)

    def _log_high_frequency_warning(self, topic_name: str) -> None:
        """Log a warning once per second when a topic exceeds UPTIME_FREQUENCY Hz."""
        now_ns = self.node.get_clock().now().nanoseconds

        if topic_name not in self._logged:
            self._logged[topic_name] = [now_ns]
            return

        timestamps = self._logged[topic_name]

        # If more than 1 second has passed since the first logged timestamp
        if timestamps and (now_ns - timestamps[0] >= 1_000_000_000):
            self.print_warning(f"Node receiving '{topic_name}' data at over {UPTIME_FREQUENCY} Hz. Check for infinite loops or excessive publishing. Possibly stopped receiving data.")
            self._logged[topic_name] = []  # reset for next second

        self._logged[topic_name].append(now_ns)
        
    def shutdown(self) -> None:
        """Gracefully shut down the scheduler, all tasks, and the ROS node."""
        self._running = False

        # Wait for watcher thread to exit
        while self._thread.is_alive():
            time.sleep(0.1)
        self._thread.join()

        self.print_warning(f"{self.node.get_name()} node has shutdown.")
        self.node.destroy_node()
        rclpy.shutdown()
    
    def stop(self, device: Threading) -> None:
        """Stop watching a device and shut down the watchdog if no devices remain."""
        self.remove_device(device)

        if not self._devices:
            self.shutdown()

# =============================================================================
# GLOBAL DEBUGGER (Lazy Initialization)
# =============================================================================

_global_watchdog_instance: "Watchdog | None" = None


class _GlobalWatchdogProxy:
    """Proxy object that creates the real Watchdog **only** on first use.

    This gives you the exact same convenient global access you had before:
        global_watchdog.add_device(...)
        global_watchdog.remove_device(...)

    But the ROS node + watcher thread are **not** started until the first
    time you actually touch `global_watchdog`.
    """
    def __getattr__(self, name: str):
        global _global_watchdog_instance
        if _global_watchdog_instance is None:
            _global_watchdog_instance = Watchdog()
        return getattr(_global_watchdog_instance, name)

    def __setattr__(self, name: str, value):
        # Allow setting attributes directly on the proxy if needed
        global _global_watchdog_instance
        if _global_watchdog_instance is None:
            _global_watchdog_instance = Watchdog()
        return setattr(_global_watchdog_instance, name, value)


# Public global instance — usage stays EXACTLY the same as before
global_watchdog = _GlobalWatchdogProxy()


# Optional: explicit getter (recommended for new code)
def get_watchdog() -> "Watchdog":
    """Get (and lazily create) the global watchdog instance."""
    global _global_watchdog_instance
    if _global_watchdog_instance is None:
        _global_watchdog_instance = Watchdog()
    return _global_watchdog_instance