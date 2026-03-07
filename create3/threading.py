#
# ROS MultiThreading Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread
from typing import Callable
from colorama import init, Fore, Style 

from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .tools import RosTools
from .models import Debug, SubscriberTopics, PublisherTopics

init(autoreset=True)

class _Threading():
    def __init__(self, node: Node):
        self.node = node

        self.debug = Debug()

        # Creates a exclusive callback group so not to interrupt the other callbacks.
        publishCbGroup = MutuallyExclusiveCallbackGroup()
        self._subscriptionCbGroup = MutuallyExclusiveCallbackGroup()
        self._actionCbGroup = MutuallyExclusiveCallbackGroup()
        self._otherCbGroup = MutuallyExclusiveCallbackGroup()
        
        self.node.create_timer(0.05, self._publishHandler, callback_group=publishCbGroup)
        
    def print(self, msg):
        """Prints a value to node Info stream"""
        self.node.get_logger().info(RosTools.objectTOString(msg))

    def print_good(self, msg):
        """Prints a value to node Info stream as Green"""
        self.node.get_logger().info(Fore.GREEN + RosTools.objectTOString(msg))

    def print_fatal(self, msg):
        """Prints a value to node Fatal stream"""
        self.node.get_logger().fatal(RosTools.objectTOString(msg))

    def print_error(self, msg):
        """Prints a value to node Error stream"""
        self.node.get_logger().error(RosTools.objectTOString(msg))

    def print_warning(self, msg):
        """Prints a value to node Warning stream"""
        self.node.get_logger().warning(RosTools.objectTOString(msg))

    def time(self) -> int:
        """Returns the current timestamp for ROS"""
        return self.node.get_clock().now().nanoseconds

    def get_name(self):
        """Returns the Node name."""
        return self.node.get_name()

    def update_uptime(self, topic_name: str):
        """Updates the timestamps for each subscription callback."""
        if not topic_name in self.debug.uptime:
            self.debug.uptime[topic_name] = [0, 0, 0, 0, 0] # [last_time, frequency, min_freq, max_freq, total_calls]
        self.debug.uptime[topic_name][1] = int(1 / ((self.time() - self.debug.uptime.get(topic_name, 0)[0]) / 1000000000)) # frequency = 1 / (current_time - last_time)
        self.debug.uptime[topic_name][0] = self.time() # last_time = current_time
        self.debug.uptime[topic_name][2] = min(self.debug.uptime[topic_name][1], self.debug.uptime[topic_name][2] if self.debug.uptime[topic_name][2] != 0 else float('inf')) # min_freq = min(current_freq, min_freq)
        self.debug.uptime[topic_name][3] = max(self.debug.uptime[topic_name][1], self.debug.uptime[topic_name][3]) # max_freq = max(current_freq, max_freq)
        self.debug.uptime[topic_name][4] += 1 # total_calls += 1

    def start(self):
        """Spin up ROS on single thread"""
        self.print(f'{self.node.get_name()} node is initiating... Listening for Topics Sub/Pub, Services and Actions.')
        self._executor = SingleThreadedExecutor()
        self._ros_thread = Thread(target=self._spin)
        self._ros_thread.start()
    
    def shutdown(self):
        self._executor.shutdown()
        while self._ros_thread.is_alive():
            time.sleep(0.1)
        self._ros_thread.join()

        self.print_warning(f'{self.node.get_name()} node has shutdown.')
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
        self._executor.add_node(self.node)
        self._executor.spin()
                
    def _publishHandler(self):
        """Loop for checking for updates and publishing Constantly every 0.5 sec"""
        pass

class RobotThreading(_Threading):
    """Spin up ROS node using multithreading."""
    def __init__(self, node: Node):
        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Create3"

        # Hidden global callback information
        self._subscribe = SubscriberTopics.ROBOT
        
        # Hidden global publish information
        self._publish = PublisherTopics.ROBOT
        
        # Creates a exclusive callback group so not to interrupt the other callbacks.
        setWheelSpeedCbGroup = MutuallyExclusiveCallbackGroup()
        self._cmdVelCbGroup = MutuallyExclusiveCallbackGroup()
        
        # Creates a timer that will loop every 0.499s and set wheel speeds if exist
        node.create_timer(0.05, self._setWheelSpeedHandler, callback_group=setWheelSpeedCbGroup)
        
        # Declare Tools
        self.tools = RosTools.ROBOT

    def shutdown(self):
        super().shutdown()
    
    def _setWheelSpeedHandler(self):
        """Loop for setting wheel speeds Constantly every 0.5 sec"""
        pass

class RpiThreading(_Threading):
    """Spin up ROS node using multithreading."""
    def __init__(self, node: Node):
        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Raspberry Pi"

        # Hidden global callback information
        self._subscribe = SubscriberTopics.RPI
        
        # Hidden global publish information
        self._publish = PublisherTopics.RPI
        
        # Declare Tools
        self.tools = RosTools.RPI

class PcThreading(_Threading):
    """Spin up ROS node using multithreading."""
    def __init__(self, node: Node):
        super().__init__(node) # trigger original code before it gets overwritten
        self.node._logger.name = "Remote PC"

        # Hidden global callback information
        self._subscribe = SubscriberTopics.PC
        
        # Hidden global publish information
        self._publish = PublisherTopics.PC

        # Declare Tools
        self.tools = RosTools.PC
        