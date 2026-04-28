#
# Threading for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread
from typing import Any, Callable

from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.executors import SingleThreadedExecutor

from create3.models import Debug
from .logger import Logger

class Threading(Logger):
    """ROS node threading helper with background spinning, timing utilities,
    topic uptime/frequency tracking, and delayed one-shot callbacks.

    Inherits from `Logger` so all colored logging methods (`print_healthy`,
    `print_error`, etc.) are available.
    """

    def __init__(self, node: Node) -> None:
        """Initialize threading support for a ROS node.

        Parameters
        ----------
        node : Node
            The rclpy node this helper will manage.
        """
        super().__init__(node)          # properly initialize parent Logger
        self.debug = Debug()            # debug statistics container

    def time(self) -> int:
        """Return the current ROS clock time in nanoseconds."""
        return self.node.get_clock().now().nanoseconds

    def get_name(self) -> str:
        """Return the name of this ROS node (useful for logging)."""
        return self.node.get_name()

    def update_uptime(self, topic_name: str) -> None:
        """Update frequency statistics for a topic (used by debug tools).

        Tracks: last timestamp, current frequency (Hz), min frequency,
        max frequency, and total message count.
        """
        if topic_name not in self.debug.uptime:
            self.debug.uptime[topic_name] = [0, 0, 0, 0, 0]  # [last_ns, freq, min_freq, max_freq, total_calls]

        uptime = self.debug.uptime[topic_name]
        current_ns = self.time()

        # Frequency = 1 / Δt (in seconds)
        if uptime[0] != 0:
            dt_sec = (current_ns - uptime[0]) / 1_000_000_000
            uptime[1] = int(1.0 / dt_sec) if dt_sec > 0 else 0
        else:
            uptime[1] = 0

        # Update last timestamp
        uptime[0] = current_ns

        # Update min/max frequency (ignore first sample where min is still 0)
        if uptime[2] == 0:
            uptime[2] = uptime[1]
        else:
            uptime[2] = min(uptime[1], uptime[2])
        uptime[3] = max(uptime[1], uptime[3])

        # Total calls
        uptime[4] += 1

    def start(self) -> None:
        """Start ROS spinning in a background thread.

        This allows the main thread to continue while the node processes
        topics, services, and actions.
        """
        self.print(f"{self.get_name()} node is initiating... "
                   "Listening for Topics, Services and Actions.")

        self._executor = SingleThreadedExecutor()
        self._ros_thread = Thread(target=self._spin, daemon=True)
        self._ros_thread.start()

    def shutdown(self) -> None:
        """Gracefully stop the ROS spinning thread and destroy the node."""
        self._executor.shutdown()

        # Wait for the thread to finish cleanly
        while self._ros_thread.is_alive():
            time.sleep(0.1)
        self._ros_thread.join()

        self.print_warning(f"{self.get_name()} node has shutdown.")
        self.node.destroy_node()

    def delay_callback(self, delay_time: float | int, callback: Callable, *args: Any, **kwargs: Any) -> Timer:
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
                self.node.destroy_timer(timer)
                timer = None

            try:
                callback(*args, **kwargs)
            except Exception as e:  # noqa: BLE001
                self.print_error(f"Delayed callback failed: {e}")

        # Create the timer (one-shot)
        timer = self.node.create_timer(delay_time, one_shot_wrapper)
        return timer

    def _spin(self) -> None:
        """Internal method that runs the ROS executor in a background thread.

        Do not call this directly — use `start()` instead.
        """
        self._executor.add_node(self.node)
        self._executor.spin()