#
# Goal Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.task import Future

if TYPE_CHECKING:
    from create3.ros.robot import ActionClient

def goal_response_callback(action_client: "ActionClient", future: Future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        action_client.print_warning(f"Goal rejected: {goal_handle}")
        return

    # Optionally request the result future (if you care about completion status)
    result_future: Future = goal_handle.get_result_async()
    result_future.add_done_callback(lambda msg: get_result_callback(action_client, msg))

def get_result_callback(action_client: "ActionClient", future: Future):
    result = future.result().result
    if not result:
        action_client.print_error(f"Action failed: {result}")