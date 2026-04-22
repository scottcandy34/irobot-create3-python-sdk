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

def odom_callback(subscriber: "Subscriber", odom: Odometry):
    # Handles returned odometry from robot and saves it locally
    subscriber.update_uptime(subscriber._odom.topic_name)

    position = Position()
    position.x = odom.pose.pose.position.x * 100 # convert to centimeters
    position.y = odom.pose.pose.position.y * 100 # convert to centimeters
    turn = odom.pose.pose.orientation
    position.angle = math.degrees(tools.coords.convert_to_euler(turn.x, turn.y, turn.z, turn.w).yaw_z) # Convert quaternion rotation to euler angles to get z angle and convert to degrees
    subscriber._subscription_msgs.position = position
    
def ir_intensity_callback(subscriber: "Subscriber", ir: IrIntensityVector):
    # Get individual values from the message
    subscriber.update_uptime(subscriber._ir_intensity.topic_name)

    sensor_1 = ir.readings[0].value
    sensor_2 = ir.readings[1].value
    sensor_3 = ir.readings[2].value
    sensor_4 = ir.readings[3].value
    sensor_5 = ir.readings[4].value
    sensor_6 = ir.readings[5].value
    sensor_7 = ir.readings[6].value
    
    # Save sensors globally in a list
    subscriber._subscription_msgs.ir_values = [sensor_1, sensor_2, sensor_3, sensor_4, sensor_5, sensor_6, sensor_7]
    
def hazard_detection_callback(subscriber: "Subscriber", hazards: HazardDetectionVector):
    subscriber.update_uptime(subscriber._hazard_detection.topic_name)
    
    subscriber._subscription_msgs.bumpers = HazardBumper()
    subscriber._subscription_msgs.cliff = HazardCliff()
    
    # Checks hazard detections and sets corresponding object values
    hazards: list[HazardDetection] = hazards.detections
    for hazard in hazards:
        if hazard.type == 1:
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
        
        elif hazard.type == 2:
            match hazard.header.frame_id:
                case "cliff_front_left":
                    subscriber._subscription_msgs.cliff.front_left = True
                case "cliff_front_right":
                    subscriber._subscription_msgs.cliff.front_right = True
                case "cliff_side_left":
                    subscriber._subscription_msgs.cliff.side_left = True
                case "cliff_side_right":
                    subscriber._subscription_msgs.cliff.side_right = True
                    
def interface_buttons_callback(subscriber: "Subscriber", buttons: InterfaceButtons):
    subscriber.update_uptime(subscriber._interface_buttons.topic_name)
    
    subscriber._subscription_msgs.buttons.button_1 = buttons.button_1.is_pressed
    subscriber._subscription_msgs.buttons.button_power = buttons.button_power.is_pressed
    subscriber._subscription_msgs.buttons.button_2 = buttons.button_2.is_pressed
    
def battery_state_callback(subscriber: "Subscriber", battery: BatteryState):
    subscriber.update_uptime(subscriber._battery_state.topic_name)

    subscriber._subscription_msgs.battery = battery.percentage * 100 # convert to percentage
    
    if subscriber._subscription_msgs.battery <= 10.0:
        subscriber.print_warning(f"Battery low: {subscriber._subscription_msgs.battery}% remaining.")
    
def imu_callback(subscriber: "Subscriber", imu: Imu):
    subscriber.update_uptime(subscriber._imu.topic_name)

    subscriber._subscription_msgs.acceleration.x = imu.linear_acceleration.x
    subscriber._subscription_msgs.acceleration.y = imu.linear_acceleration.y
    subscriber._subscription_msgs.acceleration.z = imu.linear_acceleration.z
    
def dock_status_callback(subscriber: "Subscriber", status: DockStatus):
    subscriber.update_uptime(subscriber._dock_status.topic_name)
    
    subscriber._subscription_msgs.dockingValues.dock_visible = status.dock_visible
    subscriber._subscription_msgs.dockingValues.is_docked = status.is_docked
    
def ir_opcode_callback(subscriber: "Subscriber", irOpcode: IrOpcode):
    # Checks dock sensors and sets corresponding object values
    subscriber.update_uptime(subscriber._ir_opcode.topic_name)

    subscriber._subscription_msgs.dockingValues.sensor = irOpcode.sensor
    match irOpcode.opcode:
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
