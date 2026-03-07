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

from .rclpy import rclpy
from .threading import _Threading

UPTIME_FREQUENCY = 100 # in Hz
DEBUGGER_INTERVAL = 2 # in Hz

colorama.init(autoreset=True)

class NodeTesting():
    def __init__(self, node: Node):
        self._node = node

    def subscription(self, interface: Subscription) -> bool:
        pub_info = self._node.get_publishers_info_by_topic(interface.topic_name)
        if len(pub_info) == 0:
            return False
        return True
    
    def publisher(self, interface: Publisher) -> bool:
        sub_info = self._node.get_subscriptions_info_by_topic(interface.topic_name)
        if len(sub_info) == 0:
            return False
        return True
    
    def action_client(self, interface: ActionClient) -> bool:
        if not interface.server_is_ready():
            return False
        return True
    
    def service_client(self, interface: Client) -> bool:
        if not interface.service_is_ready():
            return False
        return True

class RclpyDebugger():
    """A global debugger for handling Create3 nodes."""
    def __init__(self):
        rclpy.init()
        self.node: Node = rclpy.create_node('ros_debugger')
        self.node._logger.name = "Debugger"

        self.node.get_logger().info(f'{self.node.get_name()} node is initiating... Watching Topics Sub/Pub, Services and Actions.')

        self._devices: list[_Threading] = []
        self._validated: dict[str, bool] = {}
        self._logged: dict[str, list[int]] = {}

        self._thread = Thread(target=self._watcher)
        self._thread.start()

    def add_device(self, device: _Threading):
        self._devices.append(device)
        self._validated.update(device.debug.isAlive())

    def remove_device(self, device: _Threading):
        for index, obj in enumerate(self._devices):
            if obj.get_name() == device.get_name():
                self._devices.pop(index)
                break

    def print(self, msg: str):
        self.node.get_logger().info(Fore.GREEN + msg)

    def print_warn(self, msg: str):
        self.node.get_logger().warn(msg)
    
    def print_error(self, msg: str):
        self.node.get_logger().error(msg)

    def _watcher(self):
        # Wait for first device to connect
        while not self._devices:
            time.sleep(0.1)

        # Checks each device in list
        while self._devices:
            # Check each attached device
            for device in self._devices:
                test = NodeTesting(self.node)

                # Check each subscription topic
                for subscription in device.debug.subscriptions:
                    topic_name = subscription.topic_name

                    exist = test.subscription(subscription)
                    if not exist and self._validated.get(topic_name, True):
                        self._validated[topic_name] = False
                        self.print_error(f'Topic publisher \'{topic_name}\' is not available.')

                    elif exist and not self._validated.get(topic_name, True):
                        self._validated[topic_name] = True
                        self.print(f'Topic publisher \'{topic_name}\' is now available.')

                    elif topic_name in device.debug.uptime and device.debug.uptime[topic_name][1] >= UPTIME_FREQUENCY:
                        if not topic_name in self._logged:
                            self._logged[topic_name] = [self.node.get_clock().now().nanoseconds]
                        else:
                            if self._logged[topic_name][-1] - self._logged[topic_name][0] >= 1000000000:
                                self.print_warn(f'Node receiving \'{topic_name}\' data at over {UPTIME_FREQUENCY} Hz. Check for infinite loops or excessive publishing. Possibly stopped receiving data.')
                                self._logged[topic_name] = []
                            self._logged[topic_name].append(self.node.get_clock().now().nanoseconds)

                # Check each publisher topic
                for publisher in device.debug.publishers:
                    topic_name = publisher.topic_name

                    exist = test.publisher(publisher)
                    if not exist and self._validated.get(topic_name, True):
                        self._validated[topic_name] = False
                        self.print_error(f'Topic subscription \'{topic_name}\' is not available.')

                    elif exist and not self._validated.get(topic_name, True):
                        self._validated[topic_name] = True
                        self.print(f'Topic subscription \'{topic_name}\' is now available.')

                # Check each action client
                for action in device.debug.actions:
                    action_name = action._action_name

                    exist = test.action_client(action)
                    if not exist and self._validated.get(action_name, True):
                        self._validated[action_name] = False
                        self.print_error(f'Action server \'{action_name}\' is not available.')

                    elif exist and not self._validated.get(action_name, True):
                        self._validated[action_name] = True
                        self.print(f'Action server \'{topic_name}\' is now available.')
                
                # Check each service client
                for service in device.debug.services:
                    service_name = service.service_name

                    exist = test.service_client(service)
                    if not exist and self._validated.get(service_name, True):
                        self._validated[service_name] = False
                        self.print_error(f'Service \'{service_name}\' is not available.')

                    elif exist and not self._validated.get(service_name, True):
                        self._validated[service_name] = True
                        self.print(f'service \'{topic_name}\' is now available.')

            time.sleep(1 / DEBUGGER_INTERVAL)

    def stop(self, device: _Threading):
        self.remove_device(device)
        if len(self._devices) == 0:
            while self._thread.is_alive():
                time.sleep(0.1)
            self._thread.join()

            self.print_warn(f'{self.node.get_name()} node has shutdown.')
            self.node.destroy_node()
            rclpy.shutdown()
