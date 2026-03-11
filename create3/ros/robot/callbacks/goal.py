#
# Goal Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.task import Future

from create3.utils import Threading

class ActionHandler(Threading if TYPE_CHECKING else object):
    """Handles callbacks for action goals."""

    def _goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.print_warning(f"Goal rejected: {goal_handle}")
            return

        # Optionally request the result future (if you care about completion status)
        result_future: Future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future):
        result = future.result().result
        if not result:
            self.print_error(f"Action failed: {result}")