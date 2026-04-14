#
# Task Schedular Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread
import colorama
from colorama import Fore, Style

from rclpy.timer import Timer
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from . import rclpy
from .ros_threading import Threading
from create3.models.companion import DetectedShapes, Tasks as CompanionTasks
from create3.utils import companion as tools
from create3.models.companion import Subscribe as CompanionSubs
from create3.models.robot import Subscribe as RobotSubs
from create3.models.remote import Subscribe as RemoteSubs

colorama.init(autoreset=True)

class TaskSchedular():
    """
    Class to manage and execute tasks for the iRobot Create3. Tasks are added with a specified frequency, 
    and the Schedular will handle the execution of the tasks and manage the output. Devices must be added 
    to the Schedular before adding tasks that require them. The Schedular will automatically check for 
    required devices when adding a task, and will not add the task if the required devices are not present. 
    The Schedular can be shutdown to stop all tasks from running and to clean up resources.
    """

    def __init__(self):
        rclpy.init()
        self.node: Node = rclpy.create_node('task_schedular')
        self.node._logger.name = "Schedular"

        self.node.get_logger().info(f'{self.node.get_name()} node is initiating... Waiting for tasks.')

        self._devices: list[Threading] = []
        self._tasks: dict[str, Timer] = {}
        self._outputs: dict[str, any] = {}

        self._executor = SingleThreadedExecutor()
        self._thread = Thread(target=self._spin)
        self._thread.start()

    def add_device(self, device: Threading):
        """Add a device to the Schedular to watch for tasks. Devices must be added before adding tasks that require them."""
        self._devices.append(device)

    def remove_device(self, device: Threading) -> bool:
        """Remove a device from the Schedular."""
        for index, obj in enumerate(self._devices):
            if obj.get_name() == device.get_name():
                self._devices.pop(index)
                return True
        return False

    def _get_task_callback(self, task: CompanionTasks) -> callable:
        task_name: str = task.name.lower()
        match task:
            case CompanionTasks.WALL_DETECTION:
                return self._wall_detection_task
            case _:
                self.print_error(f'{task_name} is not found as a executable task.')
                return None
            
    def _check_for_devices(self, task: CompanionTasks) -> bool:
        task_name: str = task.name.lower()
        match task:
            case CompanionTasks.WALL_DETECTION:
                if self._get_companion_subscriptions() is None and self._get_robot_subscriptions() is None:
                    self.print_warn(f'{task_name} task requires the Robot and Companion nodes to be added to the Schedular.')
                    return False
                if self._get_companion_subscriptions() is None:
                    self.print_warn(f'{task_name} task requires the Companion node to be added to the Schedular.')
                    return False
                if self._get_robot_subscriptions() is None:
                    self.print_warn(f'{task_name} task requires the Robot node to be added to the Schedular.')
                    return False
                return True
            case _:
                self.print_error(f'{task_name} is not found as a executable task.')
                return False

    def add_task(self, task: CompanionTasks, frequency: float = 20.0):
        """Add a task to the Schedular with a specified frequency. The Schedular will automatically check for required devices before adding the task."""
        task_name = task.name.lower()
        if not self._check_for_devices(task):
            return
        
        if not task_name in self._tasks:
            self._tasks[task_name] = self.node.create_timer(1.0 / frequency, self._get_task_callback(task))
            self.print(f'Task Schedular added {task_name} task.')
        else:
            self.print_warn(f'Can not have more than 1 of the same task: {task_name}')

    def remove_task(self, task: CompanionTasks):
        """Remove a task from the Schedular."""
        task_name = task.name.lower()
        if task_name in self._tasks:
            self._tasks[task_name].destroy()
            self._tasks.pop(task_name)
            self.print(f'Task Schedular removed {task_name} task.')
        else:
            self.print_warn(f'{task_name} task does not exist. Can not remove.')

    def _get_robot_subscriptions(self) -> RobotSubs:
        for device in self._devices:
            if device.node.get_name() == 'create3_robot':
                return device._subscription_msgs

        return None
            
    def _get_companion_subscriptions(self) -> CompanionSubs:
        for device in self._devices:
            if device.node.get_name() == 'create3_companion':
                return device._subscription_msgs

        return None
    
    def _get_remote_subscriptions(self) -> RemoteSubs:
        for device in self._devices:
            if device.node.get_name() == 'create3_remote':
                return device._subscription_msgs

        return None

    def print(self, msg: str):
        """Print a message to the console with the Schedular's logger"""
        self.node.get_logger().info(Fore.CYAN + msg)

    def print_warn(self, msg: str):
        """Print a warning message to the console with the Schedular's logger"""
        self.node.get_logger().warn(msg)
    
    def print_error(self, msg: str):
        """Print an error message to the console with the Schedular's logger"""
        self.node.get_logger().error(msg)

    def _wall_detection_task(self):
        """Task callback function for wall detection. Uses the Lidar data to find walls and segments, and stores the output in a dictionary with the task name as the key."""
        companion = self._get_companion_subscriptions()
        robot = self._get_robot_subscriptions()

        detected_shapes = DetectedShapes()
        
        detected_shapes.coords = [(tools.lidar.get_coords(companion.lidar, index, robot.position)) for index in range(companion.lidar.size())]
        detected_shapes.walls = tools.lidar.find_lines_and_segments([point for point in detected_shapes.coords if point != None])

        task_name = CompanionTasks.WALL_DETECTION.name.lower()
        self._outputs[task_name] = detected_shapes

    def get_task_output(self, task: CompanionTasks):
        """Get the output of a task. Output is stored in a dictionary with the task name as the key."""
        task_name = task.name.lower()
        if task_name in self._outputs:
            return self._outputs[task_name]
        else:
            # self.print_warn(f'No output found for {task_name} task.')
            return None

    def shutdown(self):
        """Shutdown the Schedular and clean up resources. This will stop all tasks from running and will shutdown."""
        for task in self._tasks:
            self._tasks[task].destroy()
        self._tasks.clear()

        self._executor.shutdown()
        while self._thread.is_alive():
            time.sleep(0.1)
        self._thread.join()

        # Spin a few more times to ensure the node is fully shutdown before destroying it and shutting down ROS2
        for _ in range(5):
            if not self._executor.spin_once(timeout_sec=0.05):
                break

        self.print_warn(f'{self.node.get_name()} node has shutdown.')
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        """Internal method to spin the ROS node. Should not be called directly."""
        self._executor.add_node(self.node)
        self._executor.spin()