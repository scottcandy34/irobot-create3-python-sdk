#
# Conversion Tool Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math, pprint, colorsys

from irobot_create_msgs.msg import LedColor

class _robotValues:
    wheelDistanceApart = 23.5 # cm
    """This is the distance between the center of both wheels. 23.5cm"""
    wheelRadius = 3.6 # cm
    """This is the radius of the wheel. 3.6cm"""
    radius = 16.2
    """This is the radius of the entire bot. 16.2cm"""
    maxSpeed = 46 # cm/s
    """Possible max speed. 46cm/s"""
    
    def getIrAngle(self, index: int) -> float:
        """Return the angle from IR sensor 0 to sensor 6"""
        angle: float = None
        match index:
            case 0:
                angle = 130.6
            case 1:
                angle = 103.3
            case 2:
                angle = 85.3
            case 3:
                angle = 68.3
            case 4:
                angle = 51.05
            case 5:
                angle = 31.3
            case 6:
                angle = 0.0
                
        return angle
                
    def getLedAngle(self, index: int) -> float:
        """Return the angle for LED location"""
        angle = 0.0
        match index:
            case 0:
                angle = 60.0
            case 1:
                angle = 120.0
            case 2:
                angle = 180.0
            case 3:
                angle = 240.0
            case 4:
                angle = 300.0
            case 5:
                angle = 0.0
                
        return angle

class _ledTools:
    def adjustBrightness(self, led: LedColor, brightness: float) -> LedColor:
        """Adjusts LED Brightness from 0 to 100 percent"""
        newLed = LedColor()
        newLed.red = int(led.red * brightness)
        newLed.green = int(led.green * brightness)
        newLed.blue = int(led.blue * brightness)
        
        return newLed
        
    def adjustRotationBrightness(self, led: LedColor, rotation: float, ledDeg: float, span: float = 90) -> LedColor:
        """Return LED brightness based on rotation, position and span(how much area is covered by color)"""
        position = rotation * 360
        
        x1 = ledDeg
        if ledDeg > position and (position - 90) < (ledDeg - 360):
            x1 = ledDeg - 360
            
        x2 = ledDeg
        if ledDeg < position and (position) < (ledDeg + 360) < (position + 90):
            x2 = ledDeg + 360
        
        z1 = (x1 - (position - span)) / span # return percentage within range span
        z2 = ((position + span) - x2) / span # return percentage within range span
        
        brightness = 0.0
        if 0 < z1 < 1.0:
            brightness = z1
        elif 0 < z2 < 1.0:
            brightness = z2
        elif z1 == 1.0 or z2 == 1.0:
            brightness = 1.0
            
        return self.adjustBrightness(led, brightness)
    
    def getHuePercentage(self, percentage: float, startHue: int = 0, endHue: int = 360) -> LedColor:
        """Return LED HUE based on percentage between startHue and endHue which are in degrees"""
        hue = ((percentage * abs(endHue - startHue)) + startHue) / 360
        lightness = 0.5
        saturation = 1.0
        
        colors = colorsys.hls_to_rgb(hue, lightness, saturation)
        
        led = LedColor()
        led.red = int(colors[0] * 255)
        led.green = int(colors[1] * 255)
        led.blue = int(colors[2] * 255)
        
        return led

class _irTools:
    def getRotation(self, sensors: list[int]) -> float:
        """Returns exact rotation between IR sensors."""
        maxIndex = sensors.index(max(sensors))
        
        leftAngle = _robotValues().getIrAngle(maxIndex - 1)
        middleAngle = _robotValues().getIrAngle(maxIndex)
        rightAngle = _robotValues().getIrAngle(maxIndex + 1)
        
        angle: int = 0.0
        
        if sensors[maxIndex] > 0:
            if leftAngle and rightAngle:
                if sensors[maxIndex - 1] > sensors[maxIndex + 1]:
                    percentage = sensors[maxIndex - 1] / sensors[maxIndex]
                    angleBetween = leftAngle - middleAngle
                    angle = middleAngle + angleBetween * percentage
                else:
                    percentage = sensors[maxIndex + 1] / sensors[maxIndex]
                    angleBetween = middleAngle - rightAngle
                    angle = middleAngle - angleBetween * percentage
            elif leftAngle:
                percentage = sensors[maxIndex - 1] / sensors[maxIndex]
                angleBetween = leftAngle - middleAngle
                angle = middleAngle + angleBetween * percentage
            elif rightAngle:
                percentage = sensors[maxIndex + 1] / sensors[maxIndex]
                angleBetween = middleAngle - rightAngle
                angle = middleAngle - angleBetween * percentage
        
        return angle / 130.6
    
    def getMotionLightring(self, ir_sensors: list[int], red: int = None, green: int = None, blue: int = None) -> list[LedColor]:
        """Returns a list of LEDs that are highlighted based on IR sensors."""
        
        ledtools = _ledTools()
        if len(ir_sensors) == 7:
            rotation = self.getRotation(ir_sensors) # Returns percentage between 0.0 to 1.
            
            if red is not None and green is not None and blue is not None:
                led = LedColor(red=red, green=green, blue=blue)
            else:
                led = ledtools.getHuePercentage(rotation)
            
            rotation = ((rotation * 130.6) + 180 - 65.3) / 360
            lightring = []
            for i in range(6):
                lightring += [ledtools.adjustRotationBrightness(led, rotation, _robotValues().getLedAngle(i))]
                
            return lightring
        
        return None

class _robotTools:
    led = _ledTools()
    ir = _irTools()
    values = _robotValues()
    
    def convertToEuler(self, x: int | float, y: int | float, z: int | float, w: int | float) -> tuple[float, float, float]:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians
        
    def convertToQuaternion(self, roll_x: int | float, pitch_y: int | float, yaw_z: int | float) -> tuple[float, float, float, float]:
        """
        Convert a euler angle into quaternion (x, y, z, w)
        input in radians
        """
        w = round(math.cos(roll_x / 2)* math.cos(pitch_y / 2) * math.cos(yaw_z / 2) + math.sin(roll_x / 2) * math.sin(pitch_y / 2) * math.sin(yaw_z / 2), 15)
        x = round(math.sin(roll_x / 2)* math.cos(pitch_y / 2) * math.cos(yaw_z / 2) - math.cos(roll_x / 2) * math.sin(pitch_y / 2) * math.sin(yaw_z / 2), 15)
        y = round(math.cos(roll_x / 2)* math.sin(pitch_y / 2) * math.cos(yaw_z / 2) + math.sin(roll_x / 2) * math.cos(pitch_y / 2) * math.sin(yaw_z / 2), 15)
        z = round(math.cos(roll_x / 2)* math.cos(pitch_y / 2) * math.sin(yaw_z / 2) - math.sin(roll_x / 2) * math.sin(pitch_y / 2) * math.cos(yaw_z / 2), 15)
        
        return x, y, z, w

class RosTools:
    """Full list of useful tools for the Create3 robot."""
    robot = _robotTools()

    def objectTOString(obj) -> str:
        """Returns a pretty string with the object data"""

        if isinstance(obj, str):
            return obj

        if hasattr(obj, '__dict__'):
            data = vars(obj)
        else:
            data = obj

        return pprint.pformat(data, indent = 4, width = 80)