#
# Task Scheduler for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time
from threading import Thread, RLock
from typing import Any

from rclpy.timer import Timer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

from create3.utils import rclpy
from create3.utils import Logger
from create3.models.common import Nodes
from create3.utils import Threading
from create3.events import GlobalEventHandler
from .registry import get_task_callback, ensure_requirements, get_tasks_requiring_device

class TaskScheduler(Logger):
    """Background task scheduler for the iRobot Create3.

    Provides a central place to register devices (`Threading` objects) and
    periodic tasks. Each task runs at a configurable frequency (default 20 Hz)
    using ROS timers. Results from tasks can be retrieved via `get_task_output`.

    Runs its own ROS node in a background thread using a `MultiThreadedExecutor`.
    """

    def __init__(self) -> None:
        """Create the TaskScheduler node and start its background executor thread."""
        # Create our own node using the safe rclpy wrapper
        rclpy.init()
        node = rclpy.create_node(Nodes.TASK_SCHEDULER)

        # Initialize Logger parent
        super().__init__(node, "Scheduler")

        self.print(f"{node.get_name()} node is initiating... Waiting for tasks.")

        self._lock = RLock()
        self._devices: dict[str, Threading] = {}
        self._tasks: dict[str, Timer] = {}
        self._outputs: dict[str, Any] = {}

        self._executor = MultiThreadedExecutor()
        self._thread = Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _find_device(self, device_name: str) -> bool:
        """Return True if a device with this name is registered."""
        return device_name in self._devices

    def _get_device(self, device_name: Nodes) -> Threading | None:
        """Return the registered device or None if not found."""
        return self._devices.get(device_name)

    def add_device(self, device: Threading) -> None:
        """Add a device to the scheduler so its tasks can be managed."""
        device_name = device.get_name()

        if not self._find_device(device_name):
            self._devices[device_name] = device

    def remove_device(self, device: Threading) -> bool:
        """Remove a device AND automatically stop/remove all tasks that depend on it.
        Then auto-shutdown the scheduler if nothing is left (exactly like Watchdog)."""
        device_name = device.get_name()

        if not self._find_device(device_name):
            self.print_warning(f"{device_name} device is not found in the Scheduler. Cannot remove.")
            return False

        # Remove the device
        self._devices.pop(device_name)

        # NEW: Auto-stop every task that required this device
        for task in get_tasks_requiring_device(device_name):
            if self._find_task(task):
                self.remove_task(task)          # re-uses your existing remove_task
                self.print_notice(f"Auto-stopped dependent task '{task}' because {device_name} was removed.")

        return True

    def _find_task(self, task) -> bool:
        """Return True if the task is currently registered."""
        return str(task) in self._tasks

    def add_task(self, task, frequency: float = 20.0) -> None:
        """Add a single periodic task to the scheduler.

        The task will be executed at the given frequency (Hz) using a ROS timer.
        """
        if not ensure_requirements(self, task):
            return

        if not self._find_task(task):
            callback = get_task_callback(task)
            if callback:
                timer = self.node.create_timer(1.0 / frequency, lambda: callback(self), MutuallyExclusiveCallbackGroup())
                self._tasks[task] = timer
                self.print_notice(f"Task Schedular added {task} task.")
            else:
                self.print_error(f"{task} is not found as an executable task.")
        else:
            self.print_warning(f"Cannot have more than one of the same task: {task}")

    def add_tasks(self, tasks: list, frequency: float = 20.0) -> None:
        """Add multiple tasks at once (same frequency)."""
        for task in tasks:
            self.add_task(task, frequency)

    def remove_task(self, task) -> bool:
        """Remove a task from the scheduler."""
        if self._find_task(task):
            self._tasks[task].destroy()
            self._tasks.pop(task)
            self.print_notice(f"Task Schedular removed {task} task.")
            return True

        self.print_warning(f"{task} task does not exist. Cannot remove.")
        return False

    def clear_tasks(self) -> None:
        """Remove all tasks and clear stored outputs."""
        for timer in self._tasks.values():
            timer.destroy()
        self._tasks.clear()
        self._outputs.clear()
        self.print("Task Schedular cleared all tasks.")

    def get_task_output(self, task):
        """Return the latest output from a task (or None if not found)."""
        with self._lock:
            return self._outputs.get(task)

    def set_task_output(self, task: Any, value: Any) -> None:
        """Thread-safe write to outputs + fire event for listeners."""
        with self._lock:
            self._outputs[task] = value

        GlobalEventHandler.emit_task(task, self, value)

    def _blank_task(self) -> None:
        """Empty placeholder task (kept for API compatibility)."""
        pass

    def shutdown(self) -> None:
        """Gracefully shut down the scheduler, all tasks, and the ROS node."""
        self.clear_tasks()

        self._executor.shutdown()
        while self._thread.is_alive():
            time.sleep(0.1)
        self._thread.join()

        # Drain any remaining executor work
        for _ in range(5):
            if not self._executor.spin_once(timeout_sec=0.05):
                break

        self.print_warning(f"{self.node.get_name()} node has shutdown.")
        self.node.destroy_node()
        rclpy.shutdown()
        
    def stop(self, device: Threading) -> None:
        """Stop watching a device and shut down the scheduler if no devices remain."""
        self.remove_device(device)

        if not self._devices:
            self.shutdown()

    def _spin(self) -> None:
        """Internal background thread that runs the ROS executor."""
        self._executor.add_node(self.node)
        self._executor.spin()
        
# =============================================================================
# GLOBAL TASK SCHEDULER (Lazy Initialization)
# =============================================================================

_global_task_scheduler_instance: "TaskScheduler | None" = None


class _GlobalTaskSchedulerProxy:
    """Proxy object that creates the real TaskScheduler **only** on first use.

    This gives you the exact same convenient global access pattern as
    global_watchdog:

        from create3.scheduler.scheduler import global_task_scheduler

        global_task_scheduler.add_task(...)
        global_task_scheduler.start()
        # etc.

    No ROS node, no threads, no resource usage until you actually touch it.
    """
    def __getattr__(self, name: str):
        global _global_task_scheduler_instance
        if _global_task_scheduler_instance is None:
            _global_task_scheduler_instance = TaskScheduler()
        return getattr(_global_task_scheduler_instance, name)

    def __setattr__(self, name: str, value):
        global _global_task_scheduler_instance
        if _global_task_scheduler_instance is None:
            _global_task_scheduler_instance = TaskScheduler()
        return setattr(_global_task_scheduler_instance, name, value)


# Public global instance — usage stays clean and familiar
global_task_scheduler = _GlobalTaskSchedulerProxy()


# Optional: explicit getter (recommended for new code or when you want
# to pass parameters on first creation)
def get_task_scheduler() -> "TaskScheduler":
    """Get (and lazily create) the global task scheduler instance."""
    global _global_task_scheduler_instance
    if _global_task_scheduler_instance is None:
        _global_task_scheduler_instance = TaskScheduler()
    return _global_task_scheduler_instance