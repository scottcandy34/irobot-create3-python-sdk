from typing import TYPE_CHECKING, Any, Callable

from create3.models import Nodes
from create3.models.robot import Tasks as RobotTasks
from create3.models.remote import Tasks as RemoteTasks
from create3.models.common import Tasks as CommonTasks
from create3.models.companion import Tasks as CompanionTasks

if TYPE_CHECKING:
    from .schedular import TaskSchedular

# Import all task functions
from .tasks.companion import (
    generate_coords_task,
    wall_detection_task,
    column_detection_task,
    lidar_lightring_task,
    simple_wall_follower,
)
from .tasks.robot import ir_lightring_task
from .tasks.remote import controller_task
from .tasks.common import history_keeper_task

# Task → callback function
TASK_CALLBACKS = {
    CommonTasks.HISTORY_KEEPER: history_keeper_task,
    CompanionTasks.GENERATE_COORDS: generate_coords_task,
    CompanionTasks.WALL_DETECTION: wall_detection_task,
    CompanionTasks.COLUMN_DETECTION: column_detection_task,
    CompanionTasks.LIDAR_LIGHTRING: lidar_lightring_task,
    CompanionTasks.SIMPLE_WALL_FOLLOWER: simple_wall_follower,
    RobotTasks.IR_LIGHTRING: ir_lightring_task,
    RemoteTasks.CONTROLLER: controller_task,
}

def get_task_callback(task: Any) -> Callable | None:
    """Return the callback function for a given task name.

    Returns None if the task is not registered in TASK_CALLBACKS.
    """
    return TASK_CALLBACKS.get(task)

def check_requirements(scheduler: "TaskSchedular", task: Any) -> bool:
    """Validate that all prerequisites for a task are satisfied before adding it.

    Checks for required devices and mutually exclusive tasks.
    Logs clear warnings (or errors) when requirements are not met.

    Returns True only if the task can safely be added.
    """
    match task:
        # === Common Tasks ===
        case CommonTasks.HISTORY_KEEPER:
            return True
        # === Companion Tasks ===
        case CompanionTasks.GENERATE_COORDS:
            if not (scheduler._find_device(Nodes.CREATE3_COMPANION) and scheduler._find_device(Nodes.CREATE3_ROBOT)):
                scheduler.print_warning(f"{task} task requires both {Nodes.CREATE3_COMPANION} and {Nodes.CREATE3_ROBOT} nodes.")
                return False

        case CompanionTasks.WALL_DETECTION:
            if not scheduler._find_task(CompanionTasks.GENERATE_COORDS):
                scheduler.print_warning(f"{task} task requires the {CompanionTasks.GENERATE_COORDS} task.")
                return False

        case CompanionTasks.COLUMN_DETECTION:
            if not scheduler._find_task(CompanionTasks.GENERATE_COORDS):
                scheduler.print_warning(f"{task} task requires the {CompanionTasks.GENERATE_COORDS} task.")
                return False

        case CompanionTasks.LIDAR_LIGHTRING:
            if not (scheduler._find_device(Nodes.CREATE3_COMPANION) and scheduler._find_device(Nodes.CREATE3_ROBOT)):
                scheduler.print_warning(f"{task} task requires both {Nodes.CREATE3_COMPANION} and {Nodes.CREATE3_ROBOT} nodes.")
                return False
            if scheduler._find_task(RobotTasks.IR_LIGHTRING):
                scheduler.print_warning(f"{task} task cannot run together with {RobotTasks.IR_LIGHTRING}.")
                return False

        # === Robot Tasks ===
        case RobotTasks.IR_LIGHTRING:
            if not scheduler._find_device(Nodes.CREATE3_ROBOT):
                scheduler.print_warning(f"{task} task requires the {Nodes.CREATE3_ROBOT} node.")
                return False
            if scheduler._find_task(CompanionTasks.LIDAR_LIGHTRING):
                scheduler.print_warning(f"{task} task cannot run together with {CompanionTasks.LIDAR_LIGHTRING}.")
                return False

        # === Remote Tasks ===
        case RemoteTasks.CONTROLLER:
            if not (scheduler._find_device(Nodes.CREATE3_ROBOT) and scheduler._find_device(Nodes.CREATE3_REMOTE)):
                scheduler.print_warning(f"{task} task requires both {Nodes.CREATE3_ROBOT} and {Nodes.CREATE3_REMOTE} nodes.")
                return False

            if not scheduler._find_device(Nodes.CREATE3_COMPANION):
                scheduler.print_warning(f"{task} task works without {Nodes.CREATE3_COMPANION} (camera movement will be disabled).")
                # Still allowed to run
                return True

            if scheduler._find_task(CompanionTasks.SIMPLE_WALL_FOLLOWER):
                scheduler.print_warning(f"{task} task cannot run together with {CompanionTasks.SIMPLE_WALL_FOLLOWER}.")
                return False

        case CompanionTasks.SIMPLE_WALL_FOLLOWER:
            if not (scheduler._find_device(Nodes.CREATE3_COMPANION) and scheduler._find_device(Nodes.CREATE3_ROBOT)):
                scheduler.print_warning(f"{task} task requires both {Nodes.CREATE3_COMPANION} and {Nodes.CREATE3_ROBOT} nodes.")
                return False

            if scheduler._find_task(RemoteTasks.CONTROLLER):
                scheduler.print_warning(f"{task} task cannot run together with {RemoteTasks.CONTROLLER}.")
                return False

        # Unknown task
        case _:
            scheduler.print_error(f"{task} is not a known task.")
            return False

    return True