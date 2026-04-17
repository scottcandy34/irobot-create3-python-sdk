#
# Threading for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread
from typing import Callable

from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.executors import SingleThreadedExecutor

from create3.models import Debug
from .logger import Logger

class Threading(Logger):
    """Provides multithreading capabilities and helper functions for ROS nodes."""

    def __init__(self, node: Node):
        self.node = node
        self.debug = Debug()

    def time(self) -> int:
        """Returns the current time in nanoseconds."""
        return self.node.get_clock().now().nanoseconds

    def get_name(self):
        """Returns the name of the node for debugging purposes."""
        return self.node.get_name()

    def update_uptime(self, topic_name: str):
        """Updates the uptime and frequency stats for a given topic."""

        if not topic_name in self.debug.uptime:
            self.debug.uptime[topic_name] = [0, 0, 0, 0, 0] # [last_time, frequency, min_freq, max_freq, total_calls]
        self.debug.uptime[topic_name][1] = int(1 / ((self.time() - self.debug.uptime.get(topic_name, 0)[0]) / 1000000000)) # frequency = 1 / (current_time - last_time)
        self.debug.uptime[topic_name][0] = self.time() # last_time = current_time
        self.debug.uptime[topic_name][2] = min(self.debug.uptime[topic_name][1], self.debug.uptime[topic_name][2] if self.debug.uptime[topic_name][2] != 0 else float('inf')) # min_freq = min(current_freq, min_freq)
        self.debug.uptime[topic_name][3] = max(self.debug.uptime[topic_name][1], self.debug.uptime[topic_name][3]) # max_freq = max(current_freq, max_freq)
        self.debug.uptime[topic_name][4] += 1 # total_calls += 1

    def start(self):
        """Starts the ROS spinning in a separate thread."""

        self.print(f'{self.get_name()} node is initiating... Listening for Topics Sub/Pub, Services and Actions.')
        self._executor = SingleThreadedExecutor()
        self._ros_thread = Thread(target=self._spin)
        self._ros_thread.start()
    
    def shutdown(self):
        """Shuts down the ROS spinning thread and the node itself."""

        self._executor.shutdown()
        while self._ros_thread.is_alive():
            time.sleep(0.1)
        self._ros_thread.join()

        self.print_warning(f'{self.get_name()} node has shutdown.')
        self.node.destroy_node()

    def delay_callback(self, delay_time: float | int, callback: Callable, *args, **kwargs) -> Timer:
        """
        Schedule a one-shot delayed callback.
        
        :param delay_time: How many seconds to wait
        :type delay_time: float | int
        :param callback: The function to call after the delay
        :type callback: Callable
        :param args: Optional arguments passed to the callback
        :param kwargs: Optional arguments passed to the callback

        Returns the TImer object if you ever need to cancel it early.
        Multiple calls create independent timers that run in parallel.
        """

        timer: Timer = None # will hold reference to the timer itself
        
        def one_shot_wrapper():
            nonlocal timer
            # Destroy the timer immediately (so it never fires again)
            if timer is not None:
                self.node.destroy_timer(timer)
                timer = None

            # Call your actual code
            try:
                callback(*args, **kwargs)
            except Exception as e:
                self.print_error(f'Delayed callback failed: {e}')

        # Create the timer with the wrapper
        timer = self.node.create_timer(delay_time, one_shot_wrapper)
        return timer

    def _spin(self):
        """Internal method to spin the ROS node. Should not be called directly."""
        self._executor.add_node(self.node)
        self._executor.spin()