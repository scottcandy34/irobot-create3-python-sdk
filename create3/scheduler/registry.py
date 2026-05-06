from typing import TYPE_CHECKING, Any, Callable

from create3.models.common import Nodes
from create3.models.robot import Tasks as RobotTasks
from create3.models.remote import Tasks as RemoteTasks
from create3.models.common import Tasks as CommonTasks
from create3.models.companion import Tasks as CompanionTasks

if TYPE_CHECKING:
    from .scheduler import TaskScheduler

# Import all task functions
from .tasks import (
    generate_coords_task,
    wall_detection_task,
    lidar_lightring_task,
    simple_wall_follower_task,
    ir_lightring_task,
    controller_task,
    history_keeper_task,
)

# =============================================================================
# TASK REGISTRATION TABLES (fully declarative)
# =============================================================================

# Task → callback function
TASK_CALLBACKS: dict[Any, Callable] = {
    CommonTasks.HISTORY_KEEPER: history_keeper_task,
    CompanionTasks.GENERATE_COORDS: generate_coords_task,
    CompanionTasks.WALL_DETECTION: wall_detection_task,
    CompanionTasks.LIDAR_LIGHTRING: lidar_lightring_task,
    CompanionTasks.SIMPLE_WALL_FOLLOWER: simple_wall_follower_task,
    RobotTasks.IR_LIGHTRING: ir_lightring_task,
    RemoteTasks.CONTROLLER: controller_task,
}

# Task → list of tasks that must be started first
TASK_DEPENDENCIES: dict[Any, list[Any]] = {
    CompanionTasks.WALL_DETECTION: [CompanionTasks.GENERATE_COORDS],
}

# Task → required devices/nodes (must all be present)
TASK_REQUIREMENTS: dict[Any, dict] = {
    CommonTasks.HISTORY_KEEPER: {},
    CompanionTasks.GENERATE_COORDS: {
        "required_nodes": [Nodes.CREATE3_COMPANION, Nodes.CREATE3_ROBOT],
    },
    CompanionTasks.WALL_DETECTION: {
        "required_nodes": [Nodes.CREATE3_COMPANION, Nodes.CREATE3_ROBOT],
        "required_tasks": [CompanionTasks.GENERATE_COORDS],
    },
    CompanionTasks.LIDAR_LIGHTRING: {
        "required_nodes": [Nodes.CREATE3_COMPANION, Nodes.CREATE3_ROBOT],
    },
    CompanionTasks.SIMPLE_WALL_FOLLOWER: {
        "required_nodes": [Nodes.CREATE3_COMPANION, Nodes.CREATE3_ROBOT],
    },
    RobotTasks.IR_LIGHTRING: {
        "required_nodes": [Nodes.CREATE3_ROBOT],
    },
    RemoteTasks.CONTROLLER: {
        "required_nodes": [Nodes.CREATE3_ROBOT, Nodes.CREATE3_REMOTE],
    },
}

# Task → tasks that cannot run at the same time (mutual exclusions)
TASK_CONFLICTS: dict[Any, list[Any]] = {
    CompanionTasks.LIDAR_LIGHTRING: [RobotTasks.IR_LIGHTRING],
    CompanionTasks.SIMPLE_WALL_FOLLOWER: [RemoteTasks.CONTROLLER],
    RobotTasks.IR_LIGHTRING: [CompanionTasks.LIDAR_LIGHTRING],
    RemoteTasks.CONTROLLER: [CompanionTasks.SIMPLE_WALL_FOLLOWER],
}

# =============================================================================
# PUBLIC HELPERS
# =============================================================================


def get_task_callback(task: Any) -> Callable | None:
    """Return the callback function for a given task name."""
    return TASK_CALLBACKS.get(task)


def ensure_requirements(scheduler: "TaskScheduler", task: Any, _visited: set | None = None) -> bool:
    """Auto-start missing dependencies + validate requirements (unchanged behavior)."""
    if _visited is None:
        _visited = set()

    if task in _visited:
        scheduler.print_error(f"Circular dependency detected involving {task}")
        return False
    _visited.add(task)

    # Auto-start prerequisites
    for dep in TASK_DEPENDENCIES.get(task, []):
        if not scheduler._find_task(dep):
            scheduler.print_notice(f"Auto-starting prerequisite task '{dep}' for '{task}'")
            scheduler.add_task(dep)

    _visited.remove(task)

    # Now run the declarative validation
    return check_requirements(scheduler, task)


def check_requirements(scheduler: "TaskScheduler", task: Any) -> bool:
    """Declarative requirement checker — no more giant match statement!"""
    req = TASK_REQUIREMENTS.get(task)
    if req is None:
        scheduler.print_error(f"{task} is not a known task.")
        return False

    # 1. Check required nodes
    for node in req.get("required_nodes", []):
        if not scheduler._find_device(node):
            scheduler.print_warning(f"{task} task requires the {node} node.")
            return False

    # 2. Check required tasks (beyond auto-start)
    for required_task in req.get("required_tasks", []):
        if not scheduler._find_task(required_task):
            scheduler.print_warning(f"{task} task requires the {required_task} task.")
            return False

    # 3. Check conflicts (mutual exclusions)
    for conflicting_task in TASK_CONFLICTS.get(task, []):
        if scheduler._find_task(conflicting_task):
            scheduler.print_warning(f"{task} task cannot run together with {conflicting_task}.")
            return False

    # Special case for Controller (optional companion node warning)
    if task == RemoteTasks.CONTROLLER and not scheduler._find_device(Nodes.CREATE3_COMPANION):
        scheduler.print_warning(
            f"{task} task works without {Nodes.CREATE3_COMPANION} (camera movement will be disabled)."
        )

    return True