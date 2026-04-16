#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from sensor_msgs.msg import Joy
from nav_msgs.msg import OccupancyGrid
from yolo_msgs.msg import DetectionArray

from create3.utils import Threading
from create3.models.common import Position
from create3.utils import remote as tools
from create3.utils.common import convert_to_euler
from create3.models.remote import Subscribe, Controller, BoundingBox

class MessageHandler(Threading if TYPE_CHECKING else object):
    """Handles callback functions for remote subscriptions."""

    def __init__(self, node):
        super().__init__(node)

        # Hidden global callback information
        self._subscription_msgs = Subscribe
        """Contains the most recent messages received for each topic. Updated when a callback is triggered."""

    def _joy_callback(self, joy: Joy):
        self.update_uptime(self._joy.topic_name)

        joy_axes = joy.axes
        joy_buttons = joy.buttons

        controller = Controller()
        
        controller.left_joy.horizontal = joy_axes[0]
        controller.left_joy.vertical = joy_axes[1]
        controller.left_trigger = joy_axes[2]
        controller.right_joy.horizontal = joy_axes[3]
        controller.right_joy.vertical = joy_axes[4]
        controller.right_trigger = joy_axes[5]
        controller.dpad.left = joy_axes[6] > 0
        controller.dpad.right = joy_axes[6] < 0
        controller.dpad.up = joy_axes[7] > 0
        controller.dpad.down = joy_axes[7] < 0

        controller.buttons.x = joy_buttons[0] == 1
        controller.buttons.circle = joy_buttons[1] == 1
        controller.buttons.triangle = joy_buttons[2] == 1
        controller.buttons.square = joy_buttons[3] == 1
        controller.buttons.l1 = joy_buttons[4] == 1
        controller.buttons.r1 = joy_buttons[5] == 1
        # button 6 and 7 are also the left and right triggers but since it gets info from axes. its not used
        controller.buttons.share = joy_buttons[8] == 1
        controller.buttons.options = joy_buttons[9] == 1
        controller.buttons.ps = joy_buttons[10] == 1
        controller.left_joy.button = joy_buttons[11] == 1
        controller.right_joy.button = joy_buttons[12] == 1

        self._subscription_msgs.controller = controller
    
    def _map_callback(self, grid: OccupancyGrid):
        self.update_uptime(self._map.topic_name)

        self._subscription_msgs.map.resolution = grid.info.resolution
        self._subscription_msgs.map.data = tools.slam.occupancy_grid_to_2d(grid)

        position = Position()
        position.x = grid.info.origin.position.x * 100 # convert to centimeters
        position.y = grid.info.origin.position.y * 100 # convert to centimeters
        turn = grid.info.origin.orientation
        position.angle = math.degrees(convert_to_euler(turn.x, turn.y, turn.z, turn.w)[2]) # Convert quaternion rotation to euler angles to get z angle and convert to degrees
        self._subscription_msgs.map.origin = position

    def _yolo_detections_callback(self, yolo: DetectionArray):
        self.update_uptime(self._yolo_detections.topic_name)

        self._subscription_msgs.yolo.bounding_boxes.clear()
    
        for detection in yolo.detections:
            bounding_box = BoundingBox(
                class_id=detection.class_id,
                class_name=detection.class_name,
                score=detection.score,
                tracking_id=detection.id,
                center_x=detection.bbox.center.position.x,
                center_y=detection.bbox.center.position.y,
                theta=detection.bbox.center.theta,
                width=detection.bbox.size.x,
                height=detection.bbox.size.y,
            )
            self._subscription_msgs.yolo.bounding_boxes.append(bounding_box)