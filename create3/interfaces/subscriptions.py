#
# ROS Topic Subscription Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from typing import TYPE_CHECKING

from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, LaserScan, Range, Joy
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, HazardDetection, InterfaceButtons, DockStatus, IrOpcode

from ..threading import RobotThreading, RpiThreading, PcThreading
from ..models import Position, HazardBumper, HazardCliff, Acceleration, DockingValues, Controller, ROUNDING_VALUE

sub_qos_profile = QoSProfile(
    reliability = ReliabilityPolicy.BEST_EFFORT,
    liveliness = LivelinessPolicy.AUTOMATIC,
    durability = DurabilityPolicy.VOLATILE,
    depth = 1
)

class RobotSubscriptions(RobotThreading if TYPE_CHECKING else object):
    """Subscribe to ROS topics and handle the messages."""
    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Create Subscription
        odom = self.node.create_subscription(Odometry, 'odom', self._odom_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)
        ir_intensity = self.node.create_subscription(IrIntensityVector, 'ir_intensity', self._ir_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)
        hazard_detection = self.node.create_subscription(HazardDetectionVector, 'hazard_detection', self._hazard_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)
        interface_buttons = self.node.create_subscription(InterfaceButtons, 'interface_buttons', self._interface_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)
        battery_state = self.node.create_subscription(BatteryState, 'battery_state', self._battery_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)
        imu = self.node.create_subscription(Imu, 'imu', self._imu_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)
        dock_status = self.node.create_subscription(DockStatus, 'dock_status', self._dock_status_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)
        ir_opcode = self.node.create_subscription(IrOpcode, 'ir_opcode', self._ir_opcode_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)

        # Add topics to debugger
        self.debug.subscriptions = [odom, ir_intensity, hazard_detection, interface_buttons, battery_state, imu, dock_status, ir_opcode]

    def get_ir_proximity(self):
        """Get IR Proximity Values list between 0-6 so 7 total"""
        return self._subscribe.ir_values
        
    def get_position(self) -> Position:
        """Get robot's position and heading.
        
        Units:
            x, y: cm
            heading: deg
        """
        return self._subscribe.position # return position
    
    def get_bumpers(self) -> HazardBumper:
        """Returns an object of most recently seen bumper states."""
        return self._subscribe.bumpers
    
    def get_cliff_sensors(self) -> HazardCliff:
        """Returns object of most recently seen cliff sensor states."""
        return self._subscribe.cliff
    
    def get_touch_sensors(self) -> InterfaceButtons:
        """Returns object of most recently seen touch sensor states."""
        return self._subscribe.buttons
    
    def get_battery_level(self) -> int | float:
        """Get battery level. Returns percent."""
        return self._subscribe.battery
    
    def get_accelerometer(self) -> Acceleration:
        """Get instantaneous accelerometer values"""
        return self._subscribe.acceleration
    
    def get_docking_values(self) -> DockingValues:
        """Get Docking Values."""
        return self._subscribe.dockingValues
    
    def _odom_callback(self, odom: Odometry):
        # Handles returned odometry from robot and saves it locally
        self.update_uptime('/odom')

        position = Position()
        position.x = round(odom.pose.pose.position.x * 100, ROUNDING_VALUE) # convert to centimeters
        position.y = round(odom.pose.pose.position.y * 100, ROUNDING_VALUE) # convert to centimeters
        turn = odom.pose.pose.orientation
        position.angle = round(math.degrees(self.tools.convertToEuler(turn.x, turn.y, turn.z, turn.w)[2]), ROUNDING_VALUE) # Convert quaternion rotation to euler angles to get z angle and convert to degrees
        self._subscribe.position = position
        
    def _ir_callback(self, ir: IrIntensityVector):
        # Get individual values from the message
        self.update_uptime('/ir_intensity')

        sensor_1 = ir.readings[0].value
        sensor_2 = ir.readings[1].value
        sensor_3 = ir.readings[2].value
        sensor_4 = ir.readings[3].value
        sensor_5 = ir.readings[4].value
        sensor_6 = ir.readings[5].value
        sensor_7 = ir.readings[6].value
        
        # Save sensors globally in a list
        self._subscribe.ir_values = [sensor_1, sensor_2, sensor_3, sensor_4, sensor_5, sensor_6, sensor_7]
        
    def _hazard_callback(self, hazards: HazardDetectionVector):
        self.update_uptime('/hazard_detection')
        
        self._subscribe.bumpers = HazardBumper()
        self._subscribe.cliff = HazardCliff()
        
        # Checks hazard detections and sets corresponding object values
        hazards: list[HazardDetection] = hazards.detections
        for hazard in hazards:
            if hazard.type == 1:
                match hazard.header.frame_id:
                    case "bump_right":
                        self._subscribe.bumpers.right = True
                    case "bump_left":
                        self._subscribe.bumpers.left = True
                    case "bump_front_right":
                        self._subscribe.bumpers.front_right = True
                    case "bump_front_left":
                        self._subscribe.bumpers.front_left = True
                    case "bump_front_center":
                        self._subscribe.bumpers.front_center = True
            
            elif hazard.type == 2:
                match hazard.header.frame_id:
                    case "cliff_front_left":
                        self._subscribe.cliff.front_left = True
                    case "cliff_front_right":
                        self._subscribe.cliff.front_right = True
                    case "cliff_side_left":
                        self._subscribe.cliff.side_left = True
                    case "cliff_side_right":
                        self._subscribe.cliff.side_right = True
                        
    def _interface_callback(self, buttons: InterfaceButtons):
        self.update_uptime('/interface_buttons')
        
        self._subscribe.buttons.button_1 = buttons.button_1.is_pressed
        self._subscribe.buttons.button_power = buttons.button_power.is_pressed
        self._subscribe.buttons.button_2 = buttons.button_2.is_pressed
        
    def _battery_callback(self, battery: BatteryState):
        self.update_uptime('/battery_state')

        self._subscribe.battery = battery.percentage * 100 # convert to percentage
        
        if self._subscribe.battery <= 10.0:
            self.print_warning(f"Battery low: {self._subscribe.battery}% remaining.")
        
    def _imu_callback(self, imu: Imu):
        self.update_uptime('/imu')

        self._subscribe.acceleration.x = round(imu.linear_acceleration.x, ROUNDING_VALUE)
        self._subscribe.acceleration.y = round(imu.linear_acceleration.y, ROUNDING_VALUE)
        self._subscribe.acceleration.z = round(imu.linear_acceleration.z, ROUNDING_VALUE)
        
    def _dock_status_callback(self, status: DockStatus):
        self.update_uptime('/dock_status')
        
        self._subscribe.dockingValues.dock_visible = status.dock_visible
        self._subscribe.dockingValues.is_docked = status.is_docked
        
    def _ir_opcode_callback(self, irOpcode: IrOpcode):
        # Checks dock sensors and sets corresponding object values
        self.update_uptime('/ir_opcode')

        self._subscribe.dockingValues.sensor = irOpcode.sensor
        match irOpcode.opcode:
            case 161:
                self._subscribe.dockingValues.redBuoy = False
                self._subscribe.dockingValues.greenBuoy = False
                self._subscribe.dockingValues.forceField = True
            case 164:
                self._subscribe.dockingValues.redBuoy = False
                self._subscribe.dockingValues.greenBuoy = True
                self._subscribe.dockingValues.forceField = False
            case 165:
                self._subscribe.dockingValues.redBuoy = False
                self._subscribe.dockingValues.greenBuoy = True
                self._subscribe.dockingValues.forceField = True
            case 168:
                self._subscribe.dockingValues.redBuoy = True
                self._subscribe.dockingValues.greenBuoy = False
                self._subscribe.dockingValues.forceField = False
            case 169:
                self._subscribe.dockingValues.redBuoy = True
                self._subscribe.dockingValues.greenBuoy = False
                self._subscribe.dockingValues.forceField = True
            case 172:
                self._subscribe.dockingValues.redBuoy = True
                self._subscribe.dockingValues.greenBuoy = True
                self._subscribe.dockingValues.forceField = False
            case 173:
                self._subscribe.dockingValues.redBuoy = True
                self._subscribe.dockingValues.greenBuoy = True
                self._subscribe.dockingValues.forceField = True

class RpiSubscriptions(RpiThreading if TYPE_CHECKING else object):
    """Subscribe to ROS topics and handle the messages."""
    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Create Subscription
        scan = self.node.create_subscription(LaserScan, 'scan', self._lidar_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)
        range = self.node.create_subscription(Range, 'range', self._ultrasonic_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)

        # Add topics to debugger
        self.debug.subscriptions = [scan, range]

    def _lidar_callback(self, msg: LaserScan):
        self.update_uptime('/scan')

        if msg.ranges:
            self._subscribe.lidar.ranges = [(scan * 100 if isinstance(scan, float) else None) for scan in msg.ranges]
            self._subscribe.lidar.scan_time = msg.scan_time
            
            self._subscribe.lidar.angle_max = math.degrees(msg.angle_max)
            self._subscribe.lidar.angle_min = math.degrees(msg.angle_min)
            self._subscribe.lidar.angle_increment = math.degrees(msg.angle_increment)
            
            self._subscribe.lidar.range_max = msg.range_max * 100
            self._subscribe.lidar.range_min = msg.range_min * 100
            self._subscribe.lidar.time_increment = msg.time_increment
            
    def _ultrasonic_callback(self, msg: Range):
        self.update_uptime('/range')

        self._subscribe.ultrasonic.field_of_view = msg.field_of_view
        self._subscribe.ultrasonic.max_range = msg.max_range * 100
        self._subscribe.ultrasonic.min_range = msg.min_range * 100
        self._subscribe.ultrasonic.range = msg.range * 100

class PcSubscriptions(PcThreading if TYPE_CHECKING else object):
    """Subscribe to ROS topics and handle the messages."""
    def __init__(self, node):
        super().__init__(node) # trigger original code before it gets overwritten

        # Create Subscription
        joy = self.node.create_subscription(Joy, 'joy', self._joy_callback, sub_qos_profile, callback_group=self._subscriptionCbGroup)

        # Add topics to debugger
        self.debug.subscriptions = [joy]

    def _joy_callback(self, msg: Joy):
        self.update_uptime('/joy')

        joy_axes = msg.axes
        joy_buttons = msg.buttons

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

        self._subscribe.controller = controller