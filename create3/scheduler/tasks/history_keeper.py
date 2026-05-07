from typing import TYPE_CHECKING

from create3.models.common import Nodes
from create3.models.common import Stamped, Tasks

if TYPE_CHECKING:
    from create3.scheduler import TaskScheduler
    from create3 import RobotNode, CompanionNode, RemoteNode

def history_keeper_task(scheduler: "TaskScheduler") -> None:
    """Maintain a rolling history of the last 20 Stamped messages from all registered nodes.

    This task automatically tracks timestamped data (sensor readings, positions,
    detections, etc.) from every device (robot, companion, remote) and stores
    it in the scheduler's `_outputs` under a unique history key.

    Useful for debugging, logging, replay, or analysis of historical states.
    """
    MAX_HISTORY = 20

    # Safely get all registered devices
    companion: CompanionNode | None = scheduler._get_device(Nodes.CREATE3_COMPANION)
    robot: RobotNode | None = scheduler._get_device(Nodes.CREATE3_ROBOT)
    remote: RemoteNode | None = scheduler._get_device(Nodes.CREATE3_REMOTE)
    
    if robot is None or remote is None or companion is None:
        return

    # Collect all subscription message containers
    containers = []
    if companion is not None:
        containers.append((companion.get_name(), companion.subscriber.msgs))
    if robot is not None:
        containers.append((robot.get_name(), robot.subscriber.msgs))
    if remote is not None:
        containers.append((remote.get_name(), remote.subscriber.msgs))

    for node_name, msgs in containers:
        # Iterate through every attribute in the subscription container
        for attr_name in vars(msgs):
            data = getattr(msgs, attr_name)

            # Only track data that is wrapped in Stamped
            if isinstance(data, Stamped):
                history_key = f"{Tasks.HISTORY_KEEPER}_{node_name}_{data.name}"

                # Initialize history list if it doesn't exist
                if history_key not in scheduler._outputs:
                    scheduler.set_task_output(history_key, [])

                history: list[Stamped] = scheduler.get_task_output(history_key)

                # Append the new stamped data
                history.append(data)
                
                # Keep only the most recent MAX_HISTORY entries (rolling window)
                if len(history) > MAX_HISTORY:
                    history[:] = history[-MAX_HISTORY:]