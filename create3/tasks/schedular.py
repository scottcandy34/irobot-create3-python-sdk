#
# Task Schedular for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread
import colorama
from colorama import Fore, Style

from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import LightringLeds
from rclpy.executors import SingleThreadedExecutor

from create3.utils import rclpy
from create3.models import Nodes
from create3.utils import Threading
from create3.models.robot import Tasks as RobotTasks
from create3.models.remote import Tasks as RemoteTasks
from create3.models.companion import Tasks as CompanionTasks
from create3 import RobotNode, CompanionNode, RemoteNode

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
        self._states: dict[str, any] = {}

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

    def _get_task_callback(self, task: RobotTasks | CompanionTasks | RemoteTasks) -> callable:
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
            case RemoteTasks.CONTROLLER:
                return self._controller_task
            case _:
                self.print_error(f'{task} is not found as a executable task.')
                return self._blank_task
            
    def _check_requirements(self, task: RobotTasks | CompanionTasks | RemoteTasks) -> bool:
        match task:
            case CompanionTasks.GENERATE_COORDS:
                if self._find_device(Nodes.CREATE3_COMPANION) is None or self._find_device(Nodes.CREATE3_ROBOT) is None:
                    self.print_warn(f'{task} task requires the {Nodes.CREATE3_COMPANION} and {Nodes.CREATE3_ROBOT} nodes to be added to the Schedular.')
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
                    self.print_warn(f'{task} task requires the {Nodes.CREATE3_COMPANION} and {Nodes.CREATE3_ROBOT} nodes to be added to the Schedular.')
                    return False
                if self._find_task(RobotTasks.IR_LIGHTRING):
                    self.print_warn(f'{task} task can not be added to the Schedular with the {RobotTasks.IR_LIGHTRING} task. Please remove one of the tasks before adding the other.')
                    return False
            case RobotTasks.IR_LIGHTRING:
                if self._find_device(Nodes.CREATE3_ROBOT) is None:
                    self.print_warn(f'{task} task requires the {Nodes.CREATE3_ROBOT} node to be added to the Schedular.')
                    return False
                if self._find_task(CompanionTasks.LIDAR_LIGHTRING):
                    self.print_warn(f'{task} task can not be added to the Schedular with the {CompanionTasks.LIDAR_LIGHTRING} task. Please remove one of the tasks before adding the other.')
                    return False
            case RemoteTasks.CONTROLLER:
                if self._find_device(Nodes.CREATE3_ROBOT) is None or self._find_device(Nodes.CREATE3_REMOTE) is None:
                    self.print_warn(f'{task} task requires the {Nodes.CREATE3_ROBOT} and {Nodes.CREATE3_REMOTE} nodes to be added to the Schedular.')
                    return False
                if self._find_device(Nodes.CREATE3_COMPANION) is None:
                    self.print_warn(f'{task} task requires the {Nodes.CREATE3_COMPANION} only for moving the camera. Task will still operate normally.')
                    return True
            case _:
                self.print_error(f'{task} is not found as a executable task.')
                return False

        return True
    
    def _find_task(self, task: RobotTasks | CompanionTasks | RemoteTasks) -> bool:
        """Find a task in the Schedular. Returns True if the task is found, False otherwise."""
        return str(task) in self._tasks

    def add_task(self, task: RobotTasks | CompanionTasks | RemoteTasks, frequency: float = 20.0):
        """Add a task to the Schedular with a specified frequency. The Schedular will automatically check for required devices before adding the task."""
        if not self._check_requirements(task):
            return
        
        if not self._find_task(task):
            self._tasks[task] = self.node.create_timer(1.0 / frequency, self._get_task_callback(task))
            self.print(f'Task Schedular added {task} task.')
        else:
            self.print_warn(f'Can not have more than 1 of the same task: {task}')

    def add_tasks(self, tasks: list[RobotTasks | CompanionTasks | RemoteTasks], frequency: float = 20.0):
        """Add multiple tasks to the Schedular with a specified frequency."""
        for task in tasks:
            self.add_task(task, frequency)

    def remove_task(self, task: RobotTasks | CompanionTasks | RemoteTasks) -> bool:
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

    def _get_device(self, device: Nodes):
        if self._find_device(device):
            return self._devices[device]
        return None

    def get_task_output(self, task: RobotTasks | CompanionTasks | RemoteTasks):
        """Get the output of a task. Output is stored in a dictionary with the task name as the key."""
        if self._find_task(task):
            return self._outputs[task]
        else:
            # self.print_warn(f'No output found for {task_name} task.')
            return None

    def _find_state(self, state: States) -> bool:
        """Find a state in the Schedular. Returns True if the state is found, False otherwise."""
        return str(state) in self._states
    
    def _update_state(self, state: States, value: any):
        """Update the value of a state. State is stored in a dictionary with the state name as the key."""
        self._states[state] = value

    def _get_states(self, state: States):
        """Get the variable of a task. State is stored in a dictionary with the state name as the key."""
        if state in self._states:
            return self._states[state]
        else:
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
        companion: CompanionNode = self._get_device(Nodes.CREATE3_COMPANION)
        robot: RobotNode = self._get_device(Nodes.CREATE3_ROBOT)

        self._outputs[CompanionTasks.GENERATE_COORDS] = [(companion.tools.lidar.get_coords(companion.get_scans(), index, robot.get_position())) for index in range(companion.get_scans().size())]

    def _wall_detection_task(self):
        """Task callback function for wall detection. Uses the Lidar data to find walls and segments, and stores the output in a dictionary with the task name as the key."""
        companion: CompanionNode = self._get_device(Nodes.CREATE3_COMPANION)
        coords: list[tuple[float, float]] = self.get_task_output(CompanionTasks.GENERATE_COORDS)
        if coords is None:
            return

        self._outputs[CompanionTasks.WALL_DETECTION] = companion.tools.lidar.find_lines_and_segments([point for point in coords if point != None])

    def _column_detection_task(self):
        """Task callback function for column detection. Uses the Lidar data to find columns and segments, and stores the output in a dictionary with the task name as the key."""
        companion: CompanionNode = self._get_device(Nodes.CREATE3_COMPANION)
        coords: list[tuple[float, float]] = self.get_task_output(CompanionTasks.GENERATE_COORDS)
        if coords is None:
            return
        
        self._outputs[CompanionTasks.COLUMN_DETECTION] = companion.tools.lidar.find_circles_and_arcs([point for point in coords if point != None])

    def _controller_task(self):
        robot: RobotNode = self._get_device(Nodes.CREATE3_ROBOT)
        remote: RemoteNode = self._get_device(Nodes.CREATE3_REMOTE)
        companion: CompanionNode = self._get_device(Nodes.CREATE3_COMPANION)

        if remote.get_controller().buttons.r1: # R1 PS4
            twist_msg = remote.tools.joy.get_twist(remote.get_controller().left_joy.horizontal, remote.get_controller().left_joy.vertical)
            robot.send_twist(twist_msg)

        elif (remote.get_controller().buttons.options and robot.get_docking_values().is_docked):  # Options PS4
            self.node.get_logger().info('Undocking')
            robot.dock()
            self.node.get_logger().info('Undocking Completed')

        elif (remote.get_controller().buttons.options and not robot.get_docking_values().is_docked):  # Options PS4
            self.node.get_logger().info('Docking')
            robot.undock()
            self.node.get_logger().info('Docking Completed')
        
    def _lidar_lightring_task(self):
        companion: CompanionNode = self._get_device(Nodes.CREATE3_COMPANION)
        robot: RobotNode = self._get_device(Nodes.CREATE3_ROBOT)
        if not companion.get_scans().ranges:
            return

        robot.set_lights(companion.tools.lidar.get_motion_lightring(companion.get_scans().ranges))

    def _ir_lightring_task(self):
        robot: RobotNode = self._get_device(Nodes.CREATE3_ROBOT)
        if not robot.get_ir_proximity():
            return

        robot.set_lights(robot.tools.ir.get_motion_lightring(robot.get_ir_proximity()))

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