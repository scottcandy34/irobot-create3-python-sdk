#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from rclpy.time import Time
from sensor_msgs.msg import Joy
from nav_msgs.msg import OccupancyGrid
from yolo_msgs.msg import DetectionArray

from create3.models.common import Position, Stamped
from create3.models.remote import Map, Yolo
from create3.utils import remote as tools
from create3.utils.common.coords import convert_to_euler
from create3.models.remote import BoundingBox

if TYPE_CHECKING:
    from create3.ros.remote import Subscriber

def joy_callback(subscriber: "Subscriber", joy: Joy) -> None:
    """Handle incoming joystick (Joy) data and update the shared controller state.

    Maps the raw axes and buttons from a standard PlayStation-style controller
    into the custom `Controller` object used throughout the codebase.
    """
    subscriber.update_uptime(subscriber._joy.topic_name)

    axes = joy.axes
    buttons = joy.buttons

    controller = subscriber._subscription_msgs.controller

    # Left stick + triggers
    controller.left_joy.horizontal = axes[0]
    controller.left_joy.vertical = axes[1]
    controller.left_trigger = axes[2]

    # Right stick + triggers
    controller.right_joy.horizontal = axes[3]
    controller.right_joy.vertical = axes[4]
    controller.right_trigger = axes[5]

    # D-pad (treated as axes on many controllers)
    controller.dpad.left._update_state(axes[6] > 0)
    controller.dpad.right._update_state(axes[6] < 0)
    controller.dpad.up._update_state(axes[7] > 0)
    controller.dpad.down._update_state(axes[7] < 0)

    # Face buttons + shoulder + special buttons
    controller.buttons.x._update_state(buttons[0] == 1)
    controller.buttons.circle._update_state(buttons[1] == 1)
    controller.buttons.triangle._update_state(buttons[2] == 1)
    controller.buttons.square._update_state(buttons[3] == 1)
    controller.buttons.l1._update_state(buttons[4] == 1)
    controller.buttons.r1._update_state(buttons[5] == 1)
    controller.buttons.share._update_state(buttons[8] == 1)
    controller.buttons.options._update_state(buttons[9] == 1)
    controller.buttons.ps._update_state(buttons[10] == 1)

    # Stick press buttons
    controller.left_joy.button._update_state(buttons[11] == 1)
    controller.right_joy.button._update_state(buttons[12] == 1)

def map_callback(subscriber: "Subscriber", grid: OccupancyGrid) -> None:
    """Handle incoming occupancy grid (map) data and update the shared map state.

    Converts the grid origin from meters to centimeters and extracts the yaw
    angle from the quaternion orientation.
    """
    subscriber.update_uptime(subscriber._map.topic_name)
    
    map_ = Map()

    map_.resolution = grid.info.resolution
    map_.grid = tools.slam.occupancy_grid_to_2d(grid)

    # Origin pose (converted to cm and degrees)
    origin_pos = grid.info.origin.position
    origin_orient = grid.info.origin.orientation

    position = Position()
    position.x = origin_pos.x * 100.0
    position.y = origin_pos.y * 100.0
    euler = convert_to_euler(origin_orient.x, origin_orient.y, origin_orient.z, origin_orient.w)
    position.angle = math.degrees(euler.yaw_z)

    map_.origin = position
    
    subscriber._subscription_msgs.map = Stamped(map_, Time.from_msg(grid.header.stamp))
    
def corrected_position_callback(subscriber: "Subscriber"):
    now = Time()
    trans = subscriber._tf_buffer.lookup_transform('map', 'base_link', now)
    
    x = trans.transform.translation.x
    y = trans.transform.translation.y

def yolo_detections_callback(subscriber: "Subscriber", detection_array: DetectionArray) -> None:
    """Handle incoming YOLO detection array and convert detections into
    the internal BoundingBox format used by the rest of the system.
    """
    subscriber.update_uptime(subscriber._yolo_detections.topic_name)

    yolo = Yolo()

    for detection in detection_array.detections:
        bbox = BoundingBox(
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
        yolo.bounding_boxes.append(bbox)
        
    subscriber._subscription_msgs.yolo = Stamped(yolo, Time.from_msg(detection_array.header.stamp))