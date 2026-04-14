#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, HazardDetection, InterfaceButtons, DockStatus, IrOpcode

from create3.utils import Threading
from create3.utils import robot as tools
from create3.models.robot import Position, HazardBumper, HazardCliff, Subscribe

class MessageHandler(Threading if TYPE_CHECKING else object):
    """Handles callback functions for robot subscriptions."""

    def __init__(self, node):
        super().__init__(node)

        # Hidden global callback information
        self._subscription_msgs = Subscribe
        """Contains the most recent messages received for each topic. Updated when a callback is triggered."""

    def _odom_callback(self, odom: Odometry):
        # Handles returned odometry from robot and saves it locally
        self.update_uptime(self._odom.topic_name)

        position = Position()
        position.x = odom.pose.pose.position.x * 100 # convert to centimeters
        position.y = odom.pose.pose.position.y * 100 # convert to centimeters
        turn = odom.pose.pose.orientation
        position.angle = math.degrees(tools.convert_to_euler(turn.x, turn.y, turn.z, turn.w)[2]) # Convert quaternion rotation to euler angles to get z angle and convert to degrees
        self._subscription_msgs.position = position
        
    def _ir_intensity_callback(self, ir: IrIntensityVector):
        # Get individual values from the message
        self.update_uptime(self._ir_intensity.topic_name)

        sensor_1 = ir.readings[0].value
        sensor_2 = ir.readings[1].value
        sensor_3 = ir.readings[2].value
        sensor_4 = ir.readings[3].value
        sensor_5 = ir.readings[4].value
        sensor_6 = ir.readings[5].value
        sensor_7 = ir.readings[6].value
        
        # Save sensors globally in a list
        self._subscription_msgs.ir_values = [sensor_1, sensor_2, sensor_3, sensor_4, sensor_5, sensor_6, sensor_7]
        
    def _hazard_detection_callback(self, hazards: HazardDetectionVector):
        self.update_uptime(self._hazard_detection.topic_name)
        
        self._subscription_msgs.bumpers = HazardBumper()
        self._subscription_msgs.cliff = HazardCliff()
        
        # Checks hazard detections and sets corresponding object values
        hazards: list[HazardDetection] = hazards.detections
        for hazard in hazards:
            if hazard.type == 1:
                match hazard.header.frame_id:
                    case "bump_right":
                        self._subscription_msgs.bumpers.right = True
                    case "bump_left":
                        self._subscription_msgs.bumpers.left = True
                    case "bump_front_right":
                        self._subscription_msgs.bumpers.front_right = True
                    case "bump_front_left":
                        self._subscription_msgs.bumpers.front_left = True
                    case "bump_front_center":
                        self._subscription_msgs.bumpers.front_center = True
            
            elif hazard.type == 2:
                match hazard.header.frame_id:
                    case "cliff_front_left":
                        self._subscription_msgs.cliff.front_left = True
                    case "cliff_front_right":
                        self._subscription_msgs.cliff.front_right = True
                    case "cliff_side_left":
                        self._subscription_msgs.cliff.side_left = True
                    case "cliff_side_right":
                        self._subscription_msgs.cliff.side_right = True
                        
    def _interface_buttons_callback(self, buttons: InterfaceButtons):
        self.update_uptime(self._interface_buttons.topic_name)
        
        self._subscription_msgs.buttons.button_1 = buttons.button_1.is_pressed
        self._subscription_msgs.buttons.button_power = buttons.button_power.is_pressed
        self._subscription_msgs.buttons.button_2 = buttons.button_2.is_pressed
        
    def _battery_state_callback(self, battery: BatteryState):
        self.update_uptime(self._battery_state.topic_name)

        self._subscription_msgs.battery = battery.percentage * 100 # convert to percentage
        
        if self._subscription_msgs.battery <= 10.0:
            self.print_warning(f"Battery low: {self._subscription_msgs.battery}% remaining.")
        
    def _imu_callback(self, imu: Imu):
        self.update_uptime(self._imu.topic_name)

        self._subscription_msgs.acceleration.x = imu.linear_acceleration.x
        self._subscription_msgs.acceleration.y = imu.linear_acceleration.y
        self._subscription_msgs.acceleration.z = imu.linear_acceleration.z
        
    def _dock_status_callback(self, status: DockStatus):
        self.update_uptime(self._dock_status.topic_name)
        
        self._subscription_msgs.dockingValues.dock_visible = status.dock_visible
        self._subscription_msgs.dockingValues.is_docked = status.is_docked
        
    def _ir_opcode_callback(self, irOpcode: IrOpcode):
        # Checks dock sensors and sets corresponding object values
        self.update_uptime(self._ir_opcode.topic_name)

        self._subscription_msgs.dockingValues.sensor = irOpcode.sensor
        match irOpcode.opcode:
            case 161:
                self._subscription_msgs.dockingValues.redBuoy = False
                self._subscription_msgs.dockingValues.greenBuoy = False
                self._subscription_msgs.dockingValues.forceField = True
            case 164:
                self._subscription_msgs.dockingValues.redBuoy = False
                self._subscription_msgs.dockingValues.greenBuoy = True
                self._subscription_msgs.dockingValues.forceField = False
            case 165:
                self._subscription_msgs.dockingValues.redBuoy = False
                self._subscription_msgs.dockingValues.greenBuoy = True
                self._subscription_msgs.dockingValues.forceField = True
            case 168:
                self._subscription_msgs.dockingValues.redBuoy = True
                self._subscription_msgs.dockingValues.greenBuoy = False
                self._subscription_msgs.dockingValues.forceField = False
            case 169:
                self._subscription_msgs.dockingValues.redBuoy = True
                self._subscription_msgs.dockingValues.greenBuoy = False
                self._subscription_msgs.dockingValues.forceField = True
            case 172:
                self._subscription_msgs.dockingValues.redBuoy = True
                self._subscription_msgs.dockingValues.greenBuoy = True
                self._subscription_msgs.dockingValues.forceField = False
            case 173:
                self._subscription_msgs.dockingValues.redBuoy = True
                self._subscription_msgs.dockingValues.greenBuoy = True
                self._subscription_msgs.dockingValues.forceField = True
