#
# Task Schedular for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread

from rclpy.timer import Timer
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from create3.utils import rclpy
from create3.models import Nodes
from create3.utils import Threading

from .registry import get_task_callback, check_requirements
from create3.utils import Logger

class TaskSchedular(Logger):
    """Class to manage and execute tasks for the iRobot Create3."""

    def __init__(self):
        rclpy.init()
        self.node: Node = rclpy.create_node(Nodes.TASK_SCHEDULAR)
        self.node._logger.name = "Schedular"

        self.print(f'{self.node.get_name()} node is initiating... Waiting for tasks.')

        self._devices: dict[Threading] = {}
        self._tasks: dict[str, Timer] = {}
        self._outputs: dict[str, any] = {}

        self._executor = SingleThreadedExecutor()
        self._thread = Thread(target=self._spin)
        self._thread.start()

    def _find_device(self, device_name: str) -> bool:
        """Find a device in the Schedular by name. Returns True if found."""
        return device_name in self._devices
    
    def _get_device(self, device: Nodes):
        if self._find_device(device):
            return self._devices[device]
        return None

    def add_device(self, device: Threading):
        """Add a device to the Schedular."""
        device_name = device.get_name()
        if not self._find_device(device_name):
            self._devices[device_name] = device
            self.print_notice(f'Added {device_name} device to the Schedular.')
        else:
            self.print_warning(f'{device_name} device is already added to the Schedular. Can not add more than 1 of the same device.')

    def remove_device(self, device: Threading) -> bool:
        """Remove a device from the Schedular."""
        device_name = device.get_name()
        if self._find_device(device_name):
            self._devices.pop(device_name)
            self.print_notice(f'Removed {device_name} device from the Schedular.')
            return True
        else:
            self.print_warning(f'{device_name} device is not found in the Schedular. Can not remove.')
        return False

    def _find_task(self, task) -> bool:
        """Find a task in the Schedular."""
        return str(task) in self._tasks

    def add_task(self, task, frequency: float = 20.0):
        """Add a task to the Schedular with a specified frequency."""
        if not check_requirements(self, task):
            return
        
        if not self._find_task(task):
            callback = get_task_callback(task)
            if callback:
                self._tasks[task] = self.node.create_timer(1.0 / frequency, lambda: callback(self))
                self.print_notice(f'Task Schedular added {task} task.')
            else:
                self.print_error(f'{task} is not found as an executable task.')
        else:
            self.print_warning(f'Can not have more than 1 of the same task: {task}')

    def add_tasks(self, tasks: list, frequency: float = 20.0):
        """Add multiple tasks to the Schedular."""
        for task in tasks:
            self.add_task(task, frequency)

    def remove_task(self, task) -> bool:
        """Remove a task from the Schedular."""
        if self._find_task(task):
            self._tasks[task].destroy()
            self._tasks.pop(task)
            self.print_notice(f'Task Schedular removed {task} task.')
            return True
        else:
            self.print_warning(f'{task} task does not exist. Can not remove.')
            return False

    def clear_tasks(self):
        """Remove all tasks from the Schedular."""
        for task in self._tasks:
            self._tasks[task].destroy()
        self._tasks.clear()
        self._outputs.clear()
        self.print(f'Task Schedular cleared all tasks.')

    def get_task_output(self, task):
        """Get the output of a task."""
        if self._find_task(task):
            return self._outputs.get(task)
        return None

    def _blank_task(self):
        pass

    def shutdown(self):
        """Shutdown the Schedular and clean up resources."""
        self.clear_tasks()

        self._executor.shutdown()
        while self._thread.is_alive():
            time.sleep(0.1)
        self._thread.join()

        for _ in range(5):
            if not self._executor.spin_once(timeout_sec=0.05):
                break

        self.print_warning(f'{self.node.get_name()} node has shutdown.')
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        """Internal method to spin the ROS node."""
        self._executor.add_node(self.node)
        self._executor.spin()