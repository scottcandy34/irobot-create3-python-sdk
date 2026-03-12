#
# Debugger for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread
import colorama
from colorama import Fore, Style 

from rclpy.node import Node
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from . import rclpy
from .ros_threading import Threading

UPTIME_FREQUENCY = 100 # in Hz
DEBUGGER_INTERVAL = 2 # in Hz

colorama.init(autoreset=True)

class NodeTesting():
    """A class to test the availability of ROS interfaces on a given node."""

    def __init__(self, node: Node):
        self._node = node

    def subscription(self, interface: Subscription) -> bool:
        """Check if a subscription topic is being published to by any node."""
        pub_info = self._node.get_publishers_info_by_topic(interface.topic_name)
        if len(pub_info) == 0:
            return False
        return True
    
    def publisher(self, interface: Publisher) -> bool:
        """Check if a publisher topic is being subscribed to by any node."""
        sub_info = self._node.get_subscriptions_info_by_topic(interface.topic_name)
        if len(sub_info) == 0:
            return False
        return True
    
    def action_client(self, interface: ActionClient) -> bool:
        """Check if an action client has a server available."""
        if not interface.server_is_ready():
            return False
        return True
    
    def service_client(self, interface: Client) -> bool:
        """Check if a service client has a server available."""
        if not interface.service_is_ready():
            return False
        return True

class Debugger():
    """A class to watch the ROS interfaces and uptime of attached nodes, and print warnings or errors if they are not working as expected."""

    def __init__(self):
        rclpy.init()
        self.node: Node = rclpy.create_node('ros_debugger')
        self.node._logger.name = "Debugger"

        self.node.get_logger().info(f'{self.node.get_name()} node is initiating... Watching Topics Sub/Pub, Services and Actions.')

        self._devices: list[Threading] = []
        self._validated: dict[str, bool] = {}
        self._logged: dict[str, list[int]] = {}

        self._thread = Thread(target=self._watcher)
        self._thread.start()

    def add_device(self, device: Threading):
        """Add a device to the debugger to watch its ROS interfaces and uptime."""
        self._devices.append(device)
        self._validated.update(device.debug.isAlive())

    def remove_device(self, device: Threading):
        """Remove a device from the debugger to stop watching its ROS interfaces and uptime."""
        for index, obj in enumerate(self._devices):
            if obj.get_name() == device.get_name():
                self._devices.pop(index)
                break

    def print(self, msg: str):
        """Print a message to the console with the Debugger's logger"""
        self.node.get_logger().info(Fore.GREEN + msg)

    def print_warn(self, msg: str):
        """Print a warning message to the console with the Debugger's logger"""
        self.node.get_logger().warn(msg)
    
    def print_error(self, msg: str):
        """Print an error message to the console with the Debugger's logger"""
        self.node.get_logger().error(msg)

    def _check_interface(self, interface):
        """Check if a given interface is available on the node, and print a warning or error if it is not."""
        test = NodeTesting(self.node)
        name = ""
        type_ = ""

        if isinstance(interface, Subscription):
            exist = test.subscription(interface)
            name = interface.topic_name
            type_ = "Topic Publisher"

        elif isinstance(interface, Publisher):
            exist = test.publisher(interface)
            name = interface.topic_name
            type_ = "Topic Subscriber"

        elif isinstance(interface, ActionClient):
            exist = test.action_client(interface)
            name = interface._action_name
            type_ = "Action Server"

        elif isinstance(interface, Client):
            exist = test.service_client(interface)
            name = interface.service_name
            type_ = "Service Server"

        else:
            raise ValueError("ROS interface type not recognized.")
        
        if not exist and self._validated.get(name, True):
            self._validated[name] = False
            self.print_error(f'{type_} \'{name}\' is not available.')

        elif exist and not self._validated.get(name, True):
            self._validated[name] = True
            self.print(f'{type_} \'{name}\' is now available.')

    def _watcher(self):
        # Wait for first device to connect
        while not self._devices:
            time.sleep(0.1)

        # Checks each device in list
        while self._devices:
            # Check each attached device
            for device in self._devices:
                # Check each subscription topic
                for subscription in device.debug.subscriptions:
                    self._check_interface(subscription)

                    topic_name = subscription.topic_name
                    if topic_name in device.debug.uptime and device.debug.uptime[topic_name][1] >= UPTIME_FREQUENCY:
                        if not topic_name in self._logged:
                            self._logged[topic_name] = [self.node.get_clock().now().nanoseconds]
                        else:
                            if self._logged[topic_name][-1] - self._logged[topic_name][0] >= 1000000000:
                                self.print_warn(f'Node receiving \'{topic_name}\' data at over {UPTIME_FREQUENCY} Hz. Check for infinite loops or excessive publishing. Possibly stopped receiving data.')
                                self._logged[topic_name] = []
                            self._logged[topic_name].append(self.node.get_clock().now().nanoseconds)

                # Check each publisher topic
                for publisher in device.debug.publishers:
                    self._check_interface(publisher)

                # Check each action client
                for action in device.debug.actions:
                    self._check_interface(action)
                
                # Check each service client
                for service in device.debug.services:
                    self._check_interface(service)

            time.sleep(1 / DEBUGGER_INTERVAL)

    def stop(self, device: Threading):
        """Stop the debugger from watching a given device, and shutdown the debugger if there are no more devices to watch."""
        self.remove_device(device)
        if len(self._devices) == 0:
            while self._thread.is_alive():
                time.sleep(0.1)
            self._thread.join()

            self.print_warn(f'{self.node.get_name()} node has shutdown.')
            self.node.destroy_node()
            rclpy.shutdown()

# Initialize Debugger for global use
global_debugger = Debugger()