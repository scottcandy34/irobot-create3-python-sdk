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
from irobot_create_msgs.msg import LightringLeds
from rclpy.executors import SingleThreadedExecutor

from . import rclpy
from create3.models import Nodes
from .ros_threading import Threading
from create3.models.robot import Tasks as RobotTasks, Subscribe as RobotSubs, Publish as RobotPubs
from create3.models.remote import Subscribe as RemoteSubs, Publish as RemotePubs
from create3.models.companion import Tasks as CompanionTasks, Publish as CompanionPubs, Subscribe as CompanionSubs

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
        self.node: Node = rclpy.create_node(Nodes.TASK_SCHEDULAR)
        self.node._logger.name = "Schedular"

        self.node.get_logger().info(f'{self.node.get_name()} node is initiating... Waiting for tasks.')

        self._devices: dict[Threading] = {}
        self._tasks: dict[str, Timer] = {}
        self._outputs: dict[str, any] = {}

        self._executor = SingleThreadedExecutor()
        self._thread = Thread(target=self._spin)
        self._thread.start()

    def _find_device(self, device_name: str) -> bool:
        """Find a device in the Schedular by name. Returns the device if found, False otherwise."""
        return device_name in self._devices

    def add_device(self, device: Threading):
        """Add a device to the Schedular to watch for tasks. Devices must be added before adding tasks that require them."""
        device_name = device.get_name()
        if not self._find_device(device_name):
            self._devices[device_name] = device
            self.print(f'Added {device_name} device to the Schedular.')
        else:
            self.print_warn(f'{device_name} device is already added to the Schedular. Can not add more than 1 of the same device.')

    def remove_device(self, device: Threading) -> bool:
        """Remove a device from the Schedular."""
        device_name = device.get_name()
        if self._find_device(device_name):
            self._devices.pop(device_name)
            self.print(f'Removed {device_name} device from the Schedular.')
            return True
        else:
            self.print_warn(f'{device_name} device is not found in the Schedular. Can not remove.')
        return False

    def _get_task_callback(self, task: CompanionTasks) -> callable:
        self._outputs[task] = None

        match task:
            case CompanionTasks.GENERATE_COORDS:
                return self._generate_coords_task
            case CompanionTasks.WALL_DETECTION:
                return self._wall_detection_task
            case CompanionTasks.COLUMN_DETECTION:
                return self._column_detection_task
            case CompanionTasks.LIDAR_LIGHTRING:
                return self._lidar_lightring_task
            case RobotTasks.IR_LIGHTRING:
                return self._ir_lightring_task
            case _:
                self.print_error(f'{task} is not found as a executable task.')
                return self._blank_task
            
    def _check_requirements(self, task: CompanionTasks) -> bool:
        match task:
            case CompanionTasks.GENERATE_COORDS:
                if self._find_device(Nodes.CREATE3_COMPANION) is None or self._find_device(Nodes.CREATE3_ROBOT) is None:
                    self.print_warn(f'{task} task requires the Robot and Companion nodes to be added to the Schedular.')
                    return False
            case CompanionTasks.WALL_DETECTION:
                if not self._find_task(CompanionTasks.GENERATE_COORDS):
                    self.print_warn(f'{task} task requires the {CompanionTasks.GENERATE_COORDS} task to be added to the Schedular.')
                    return False
            case CompanionTasks.COLUMN_DETECTION:
                if not self._find_task(CompanionTasks.GENERATE_COORDS):
                    self.print_warn(f'{task} task requires the {CompanionTasks.GENERATE_COORDS.name} task to be added to the Schedular.')
                    return False
            case CompanionTasks.LIDAR_LIGHTRING:
                if self._find_device(Nodes.CREATE3_COMPANION) is None or self._find_device(Nodes.CREATE3_ROBOT) is None:
                    self.print_warn(f'{task} task requires the Companion and Robot nodes to be added to the Schedular.')
                    return False
                if self._find_task(RobotTasks.IR_LIGHTRING):
                    self.print_warn(f'{task} task can not be added to the Schedular with the {RobotTasks.IR_LIGHTRING} task. Please remove one of the tasks before adding the other.')
                    return False
            case RobotTasks.IR_LIGHTRING:
                if self._find_device(Nodes.CREATE3_ROBOT) is None:
                    self.print_warn(f'{task} task requires the Robot node to be added to the Schedular.')
                    return False
                if self._find_task(CompanionTasks.LIDAR_LIGHTRING):
                    self.print_warn(f'{task} task can not be added to the Schedular with the {CompanionTasks.LIDAR_LIGHTRING} task. Please remove one of the tasks before adding the other.')
                    return False
            case _:
                self.print_error(f'{task} is not found as a executable task.')
                return False

        return True
    
    def _find_task(self, task: CompanionTasks) -> bool:
        """Find a task in the Schedular. Returns True if the task is found, False otherwise."""
        return str(task) in self._tasks

    def add_task(self, task: CompanionTasks, frequency: float = 20.0):
        """Add a task to the Schedular with a specified frequency. The Schedular will automatically check for required devices before adding the task."""
        if not self._check_requirements(task):
            return
        
        if not self._find_task(task):
            self._tasks[task] = self.node.create_timer(1.0 / frequency, self._get_task_callback(task))
            self.print(f'Task Schedular added {task} task.')
        else:
            self.print_warn(f'Can not have more than 1 of the same task: {task}')

    def add_tasks(self, tasks: list[CompanionTasks], frequency: float = 20.0):
        """Add multiple tasks to the Schedular with a specified frequency."""
        for task in tasks:
            self.add_task(task, frequency)

    def remove_task(self, task: CompanionTasks) -> bool:
        """Remove a task from the Schedular."""
        if self._find_task(task):
            self._tasks[task].destroy()
            self._tasks.pop(task)
            self.print(f'Task Schedular removed {task} task.')
            return True
        else:
            self.print_warn(f'{task} task does not exist. Can not remove.')
            return False

    def clear_tasks(self):
        """Remove all tasks from the Schedular."""
        for task in self._tasks:
            self._tasks[task].destroy()
        self._tasks.clear()
        self._outputs.clear()
        self.print(f'Task Schedular cleared all tasks.')

    def _get_subscriptions(self, device: Nodes):
        if self._find_device(device):
            return self._devices[device]._subscription_msgs
        return None
        
    def _get_publishers(self, device: Nodes):
        if self._find_device(device):
            return self._devices[device]._publisher_msgs
        return None

    def _get_tools(self, device: Nodes):
        if self._find_device(device):
            return self._devices[device].tools
        return None

    def get_task_output(self, task: CompanionTasks):
        """Get the output of a task. Output is stored in a dictionary with the task name as the key."""
        if self._find_task(task):
            return self._outputs[task]
        else:
            # self.print_warn(f'No output found for {task_name} task.')
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

    def _blank_task(self):
        """Blank task to use as a placeholder for tasks that are not found or do not have the required devices."""
        pass

    def _generate_coords_task(self):
        """Task callback function for wall detection. Uses the Lidar data to find walls and segments, and stores the output in a dictionary with the task name as the key."""
        companion: CompanionSubs = self._get_subscriptions(Nodes.CREATE3_COMPANION)
        robot: RobotSubs = self._get_subscriptions(Nodes.CREATE3_ROBOT)
        tools = self._get_tools(Nodes.CREATE3_COMPANION)

        self._outputs[CompanionTasks.GENERATE_COORDS] = [(tools.lidar.get_coords(companion.lidar, index, robot.position)) for index in range(companion.lidar.size())]

    def _wall_detection_task(self):
        """Task callback function for wall detection. Uses the Lidar data to find walls and segments, and stores the output in a dictionary with the task name as the key."""
        tools = self._get_tools(Nodes.CREATE3_COMPANION)
        coords: list[tuple[float, float]] = self.get_task_output(CompanionTasks.GENERATE_COORDS)
        if coords is None:
            return
        # detected_shapes.interactions = [tools.lidar.predictive.circle_to_wall_distance(wall, robot.position) for wall in detected_shapes.walls]

        self._outputs[CompanionTasks.WALL_DETECTION] = tools.lidar.find_lines_and_segments([point for point in coords if point != None])

    def _column_detection_task(self):
        """Task callback function for column detection. Uses the Lidar data to find columns and segments, and stores the output in a dictionary with the task name as the key."""
        tools = self._get_tools(Nodes.CREATE3_COMPANION)
        coords: list[tuple[float, float]] = self.get_task_output(CompanionTasks.GENERATE_COORDS)
        if coords is None:
            return
        
        self._outputs[CompanionTasks.COLUMN_DETECTION] = tools.lidar.find_circles_and_arcs([point for point in coords if point != None])
    
    def _lidar_lightring_task(self):
        companion: CompanionSubs = self._get_subscriptions(Nodes.CREATE3_COMPANION)
        robot: RobotPubs = self._get_publishers(Nodes.CREATE3_ROBOT)
        tools = self._get_tools(Nodes.CREATE3_COMPANION)
        if not companion.lidar.ranges:
            return

        lightMsg = LightringLeds()
        lightMsg.override_system = True
        lightMsg.leds = tools.lidar.get_motion_lightring(companion.lidar.ranges)

        robot.lightring = lightMsg

    def _ir_lightring_task(self):
        subscription: RobotSubs = self._get_subscriptions(Nodes.CREATE3_ROBOT)
        robot: RobotPubs = self._get_publishers(Nodes.CREATE3_ROBOT)
        tools = self._get_tools(Nodes.CREATE3_ROBOT)
        if not subscription.ir_values:
            return

        lightMsg = LightringLeds()
        lightMsg.override_system = True
        lightMsg.leds = tools.ir.get_motion_lightring(subscription.ir_values)

        robot.lightring = lightMsg

    def shutdown(self):
        """Shutdown the Schedular and clean up resources. This will stop all tasks from running and will shutdown."""
        self.clear_tasks()

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