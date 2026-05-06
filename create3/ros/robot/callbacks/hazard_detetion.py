#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection

from create3.utils import common as tools
from create3.models.common import Position, Stamped
from create3.models.robot import Acceleration

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def hazard_detection_callback(subscriber: "Subscriber", vector: HazardDetectionVector) -> None:
    """Parse hazard detections (bumpers and cliffs) and update the shared state objects."""
    # Shared model objects
    bumpers = subscriber.bumpers
    cliffs = subscriber.cliff_sensors

    # 1. Start with clean "all false" local state
    bumper_states = {
        'right':        False,
        'front_right':  False,
        'front_center': False,
        'front_left':   False,
        'left':         False,
    }
    cliff_states = {
        'side_right':  False,
        'front_right': False,
        'front_left':  False,
        'side_left':   False,
    }

    # 2. Set True for every currently active hazard
    detection: HazardDetection
    for detection in vector.detections:
        frame_id = detection.header.frame_id.lower()

        # Bumper hazards
        if "bump_front_left" in frame_id:
            bumper_states['front_left'] = True
        elif "bump_front_center" in frame_id:
            bumper_states['front_center'] = True
        elif "bump_front_right" in frame_id:
            bumper_states['front_right'] = True
        elif "bump_left" in frame_id:
            bumper_states['left'] = True
        elif "bump_right" in frame_id:
            bumper_states['right'] = True

        # Cliff hazards
        elif "cliff_front_left" in frame_id:
            cliff_states['front_left'] = True
        elif "cliff_front_right" in frame_id:
            cliff_states['front_right'] = True
        elif "cliff_side_left" in frame_id:
            cliff_states['side_left'] = True
        elif "cliff_side_right" in frame_id:
            cliff_states['side_right'] = True

    # 3. Apply the complete final state to every Button (one clean pass)
    bumpers.right._update_state(bumper_states['right'])
    bumpers.front_right._update_state(bumper_states['front_right'])
    bumpers.front_center._update_state(bumper_states['front_center'])
    bumpers.front_left._update_state(bumper_states['front_left'])
    bumpers.left._update_state(bumper_states['left'])

    cliffs.side_right._update_state(cliff_states['side_right'])
    cliffs.front_right._update_state(cliff_states['front_right'])
    cliffs.front_left._update_state(cliff_states['front_left'])
    cliffs.side_left._update_state(cliff_states['side_left'])
