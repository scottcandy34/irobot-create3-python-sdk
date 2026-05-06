#
# Goal Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.task import Future

if TYPE_CHECKING:
    from create3.ros.robot import ActionClient

def goal_response_callback(action_client: "ActionClient", future: Future) -> None:
    """Handle the server's response after a goal has been sent to an action server.

    Checks whether the goal was accepted. If accepted, it automatically
    registers a callback to receive the final result when the action completes.
    """
    goal_handle = future.result()

    if not goal_handle.accepted:
        action_client.print_warning(f"Goal rejected: {goal_handle}")
        return

    # Goal accepted → request the result asynchronously
    result_future: Future = goal_handle.get_result_async()
    result_future.add_done_callback(
        lambda future_msg: get_result_callback(action_client, future_msg)
    )

def get_result_callback(action_client: "ActionClient", future: Future) -> None:
    """Handle the final result returned by the action server when the goal completes.

    Called automatically via the done callback set up in `goal_response_callback`.
    """
    result = future.result().result

    if not result:
        action_client.print_error(f"Action failed: {result}")
        return

    # Optional success logging (uncomment if desired)
    # action_client.print_healthy(f"Action completed successfully: {result}")