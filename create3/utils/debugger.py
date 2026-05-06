#
# Debugger for iRobot Create3 - Jazzy
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
from create3.models import Nodes
from .ros_threading import Threading
from .logger import Logger
from .monitored_subscription import MonitoredSubscription

UPTIME_FREQUENCY = 100 # in Hz
DEBUGGER_INTERVAL = 2 # in Hz

colorama.init(autoreset=True)

class Debugger(Logger):
    """Background ROS interface watchdog and uptime monitor.

    Monitors every attached `Threading` device for:
      • Missing publishers/subscribers, action servers, or service servers
      • Topics publishing faster than `UPTIME_FREQUENCY` Hz (possible infinite loop)

    Runs in its own background thread and uses colored logging (inherited from `Logger`).
    """

    def __init__(self) -> None:
        """Start the debugger node and its background watcher thread.

        Uses the custom `rclpy` to safely initialize only once.
        """
        # Create our own debugger node (uses the custom rclpy)
        rclpy.init()
        node = rclpy.create_node(Nodes.ROS_DEBUGGER)
        node._logger.name = "Debugger"

        # Initialize Logger parent with our node
        super().__init__(node)

        self.print(f"{node.get_name()} node is initiating... Watching Topics, Services and Actions.")

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
        """Stop watching a device (removes it from the debugger)."""
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

        while self._devices:
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

    def stop(self, device: Threading) -> None:
        """Stop watching a device and shut down the debugger if no devices remain."""
        self.remove_device(device)

        if not self._devices:
            # Wait for watcher thread to exit
            while self._thread.is_alive():
                time.sleep(0.1)
            self._thread.join()

            self.print_warning(f"{self.node.get_name()} node has shutdown.")
            self.node.destroy_node()
            rclpy.shutdown()

# Initialize Debugger for global use
global_debugger = Debugger()