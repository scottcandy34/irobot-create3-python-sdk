#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.time import Time
from yolo_msgs.msg import DetectionArray, Detection

from create3.models.common import Stamped
from create3.models.remote import Yolo
from create3.models.remote import BoundingBox

if TYPE_CHECKING:
    from create3.ros.remote import Subscriber

def yolo_detections_callback(subscriber: "Subscriber", detection_array: DetectionArray) -> None:
    """Handle incoming YOLO detection array and convert detections into
    the internal BoundingBox format used by the rest of the system.
    """
    yolo = Yolo()

    detection: Detection
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
        
    subscriber.yolo = Stamped(yolo, Time.from_msg(detection_array.header.stamp))