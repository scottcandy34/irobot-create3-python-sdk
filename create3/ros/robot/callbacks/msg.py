#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, HazardDetection, InterfaceButtons, DockStatus, IrOpcode

from create3.utils import common as tools
from create3.models.common import Position, Stamped
from create3.models.robot import Acceleration

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def odom_callback(subscriber: "Subscriber", odom: Odometry) -> None:
    """Handle incoming odometry data and update the shared subscription state.

    Converts pose from meters to centimeters and quaternion to Euler yaw (degrees).
    """
    pos = odom.pose.pose.position
    orient = odom.pose.pose.orientation

    position = Position()
    position.x = pos.x * 100.0
    position.y = pos.y * 100.0
    euler = tools.coords.convert_to_euler(orient.x, orient.y, orient.z, orient.w)
    position.angle = math.degrees(euler.yaw_z)

    subscriber.position = Stamped(position, Time.from_msg(odom.header.stamp))

def ir_intensity_callback(subscriber: "Subscriber", ir: IrIntensityVector) -> None:
    """Handle IR intensity readings and store the 7 sensor values in the shared state."""
    readings = ir.readings
    ir_values = [
        readings[0].value,
        readings[1].value,
        readings[2].value,
        readings[3].value,
        readings[4].value,
        readings[5].value,
        readings[6].value,
    ]
    
    subscriber.ir_values = Stamped(ir_values, Time.from_msg(ir.header.stamp))

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

def interface_buttons_callback(subscriber: "Subscriber", buttons: InterfaceButtons) -> None:
    """Update the state of the physical buttons on the robot."""
    subscriber.buttons.button_1._update_state(buttons.button_1.is_pressed)
    subscriber.buttons.button_power._update_state(buttons.button_power.is_pressed)
    subscriber.buttons.button_2._update_state(buttons.button_2.is_pressed)

def battery_state_callback(subscriber: "Subscriber", battery: BatteryState) -> None:
    """Update battery percentage (converted to 0–100 scale) and issue a warning when low."""
    subscriber.battery = battery.percentage * 100.0

    if subscriber.battery <= 10.0:
        subscriber.print_warning(f"Battery low: {subscriber.msgs.battery:.1f}% remaining.")

def imu_callback(subscriber: "Subscriber", imu: Imu) -> None:
    """Extract linear acceleration from the IMU and store it in the shared state."""
    acceleration = Acceleration()

    accel = imu.linear_acceleration
    acceleration.x = accel.x
    acceleration.y = accel.y
    acceleration.z = accel.z
    
    subscriber.acceleration = Stamped(acceleration, Time.from_msg(imu.header.stamp))

def dock_status_callback(subscriber: "Subscriber", status: DockStatus) -> None:
    """Update docking-related values (visible, docked)."""
    subscriber.docking_values.dock_visible._update_state(status.dock_visible)
    subscriber.docking_values.is_docked._update_state(status.is_docked)

def ir_opcode_callback(subscriber: "Subscriber", ir_opcode: IrOpcode) -> None:
    """Parse IR docking opcodes and set the corresponding buoy/force-field flags."""
    subscriber.docking_values.sensor = ir_opcode.sensor

    match ir_opcode.opcode:
        case 161:
            subscriber.docking_values.redBuoy = False
            subscriber.docking_values.greenBuoy = False
            subscriber.docking_values.forceField = True
        case 164:
            subscriber.docking_values.redBuoy = False
            subscriber.docking_values.greenBuoy = True
            subscriber.docking_values.forceField = False
        case 165:
            subscriber.docking_values.redBuoy = False
            subscriber.docking_values.greenBuoy = True
            subscriber.docking_values.forceField = True
        case 168:
            subscriber.docking_values.redBuoy = True
            subscriber.docking_values.greenBuoy = False
            subscriber.docking_values.forceField = False
        case 169:
            subscriber.docking_values.redBuoy = True
            subscriber.docking_values.greenBuoy = False
            subscriber.docking_values.forceField = True
        case 172:
            subscriber.docking_values.redBuoy = True
            subscriber.docking_values.greenBuoy = True
            subscriber.docking_values.forceField = False
        case 173:
            subscriber.docking_values.redBuoy = True
            subscriber.docking_values.greenBuoy = True
            subscriber.docking_values.forceField = True
        