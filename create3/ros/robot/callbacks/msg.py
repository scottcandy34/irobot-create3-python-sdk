#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, HazardDetection, InterfaceButtons, DockStatus, IrOpcode

from create3.utils import common as tools
from create3.models.common import Position
from create3.models.robot import HazardBumper, HazardCliff

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def odom_callback(subscriber: "Subscriber", odom: Odometry) -> None:
    """Handle incoming odometry data and update the shared subscription state.

    Converts pose from meters to centimeters and quaternion to Euler yaw (degrees).
    """
    subscriber.update_uptime(subscriber._odom.topic_name)

    pos = odom.pose.pose.position
    orient = odom.pose.pose.orientation

    position = Position()
    position.x = pos.x * 100.0
    position.y = pos.y * 100.0
    euler = tools.coords.convert_to_euler(orient.x, orient.y, orient.z, orient.w)
    position.angle = math.degrees(euler.yaw_z)

    subscriber._subscription_msgs.position = position

def ir_intensity_callback(subscriber: "Subscriber", ir: IrIntensityVector) -> None:
    """Handle IR intensity readings and store the 7 sensor values in the shared state."""
    subscriber.update_uptime(subscriber._ir_intensity.topic_name)

    readings = ir.readings
    subscriber._subscription_msgs.ir_values = [
        readings[0].value,
        readings[1].value,
        readings[2].value,
        readings[3].value,
        readings[4].value,
        readings[5].value,
        readings[6].value,
    ]

def hazard_detection_callback(subscriber: "Subscriber", hazards: HazardDetectionVector) -> None:
    """Parse hazard detections (bumpers and cliffs) and update the shared state objects."""
    subscriber.update_uptime(subscriber._hazard_detection.topic_name)

    subscriber._subscription_msgs.bumpers = HazardBumper()
    subscriber._subscription_msgs.cliff = HazardCliff()

    hazards: list[HazardDetection] = hazards.detections
    for hazard in hazards:
        if hazard.type == 1:  # bumper
            match hazard.header.frame_id:
                case "bump_right":
                    subscriber._subscription_msgs.bumpers.right = True
                case "bump_left":
                    subscriber._subscription_msgs.bumpers.left = True
                case "bump_front_right":
                    subscriber._subscription_msgs.bumpers.front_right = True
                case "bump_front_left":
                    subscriber._subscription_msgs.bumpers.front_left = True
                case "bump_front_center":
                    subscriber._subscription_msgs.bumpers.front_center = True

        elif hazard.type == 2:  # cliff
            match hazard.header.frame_id:
                case "cliff_front_left":
                    subscriber._subscription_msgs.cliff.front_left = True
                case "cliff_front_right":
                    subscriber._subscription_msgs.cliff.front_right = True
                case "cliff_side_left":
                    subscriber._subscription_msgs.cliff.side_left = True
                case "cliff_side_right":
                    subscriber._subscription_msgs.cliff.side_right = True

def interface_buttons_callback(subscriber: "Subscriber", buttons: InterfaceButtons) -> None:
    """Update the state of the physical buttons on the robot."""
    subscriber.update_uptime(subscriber._interface_buttons.topic_name)

    subscriber._subscription_msgs.buttons.button_1 = buttons.button_1.is_pressed
    subscriber._subscription_msgs.buttons.button_power = buttons.button_power.is_pressed
    subscriber._subscription_msgs.buttons.button_2 = buttons.button_2.is_pressed

def battery_state_callback(subscriber: "Subscriber", battery: BatteryState) -> None:
    """Update battery percentage (converted to 0–100 scale) and issue a warning when low."""
    subscriber.update_uptime(subscriber._battery_state.topic_name)

    subscriber._subscription_msgs.battery = battery.percentage * 100.0

    if subscriber._subscription_msgs.battery <= 10.0:
        subscriber.print_warning(
            f"Battery low: {subscriber._subscription_msgs.battery:.1f}% remaining."
        )

def imu_callback(subscriber: "Subscriber", imu: Imu) -> None:
    """Extract linear acceleration from the IMU and store it in the shared state."""
    subscriber.update_uptime(subscriber._imu.topic_name)

    accel = imu.linear_acceleration
    subscriber._subscription_msgs.acceleration.x = accel.x
    subscriber._subscription_msgs.acceleration.y = accel.y
    subscriber._subscription_msgs.acceleration.z = accel.z

def dock_status_callback(subscriber: "Subscriber", status: DockStatus) -> None:
    """Update docking-related values (visible, docked)."""
    subscriber.update_uptime(subscriber._dock_status.topic_name)

    subscriber._subscription_msgs.dockingValues.dock_visible = status.dock_visible
    subscriber._subscription_msgs.dockingValues.is_docked = status.is_docked

def ir_opcode_callback(subscriber: "Subscriber", ir_opcode: IrOpcode) -> None:
    """Parse IR docking opcodes and set the corresponding buoy/force-field flags."""
    subscriber.update_uptime(subscriber._ir_opcode.topic_name)

    subscriber._subscription_msgs.dockingValues.sensor = ir_opcode.sensor

    match ir_opcode.opcode:
        case 161:
            subscriber._subscription_msgs.dockingValues.redBuoy = False
            subscriber._subscription_msgs.dockingValues.greenBuoy = False
            subscriber._subscription_msgs.dockingValues.forceField = True
        case 164:
            subscriber._subscription_msgs.dockingValues.redBuoy = False
            subscriber._subscription_msgs.dockingValues.greenBuoy = True
            subscriber._subscription_msgs.dockingValues.forceField = False
        case 165:
            subscriber._subscription_msgs.dockingValues.redBuoy = False
            subscriber._subscription_msgs.dockingValues.greenBuoy = True
            subscriber._subscription_msgs.dockingValues.forceField = True
        case 168:
            subscriber._subscription_msgs.dockingValues.redBuoy = True
            subscriber._subscription_msgs.dockingValues.greenBuoy = False
            subscriber._subscription_msgs.dockingValues.forceField = False
        case 169:
            subscriber._subscription_msgs.dockingValues.redBuoy = True
            subscriber._subscription_msgs.dockingValues.greenBuoy = False
            subscriber._subscription_msgs.dockingValues.forceField = True
        case 172:
            subscriber._subscription_msgs.dockingValues.redBuoy = True
            subscriber._subscription_msgs.dockingValues.greenBuoy = True
            subscriber._subscription_msgs.dockingValues.forceField = False
        case 173:
            subscriber._subscription_msgs.dockingValues.redBuoy = True
            subscriber._subscription_msgs.dockingValues.greenBuoy = True
            subscriber._subscription_msgs.dockingValues.forceField = True
        