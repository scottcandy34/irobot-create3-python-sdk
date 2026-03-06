#
# Conversion Tool Examples for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math, pprint, colorsys, random

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import LedColor

from .objects import Position

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

class _lineCalculations:
    def fit_line(self, points: list[tuple[float, float]]) -> tuple[float, float]:
        """Fit a line y = mx + b to the points using least squares."""
        x = [p[0] for p in points]
        y = [p[1] for p in points]
        n = len(points)
        sum_x = sum(x)
        sum_y = sum(y)
        sum_xy = sum(xi * yi for xi, yi in zip(x, y))
        sum_xx = sum(xi * xi for xi in x)
        denominator = n * sum_xx - sum_x**2
        if abs(denominator) < 1e-10:
            raise ValueError("Vertical line detected")
        m = (n * sum_xy - sum_x * sum_y) / denominator
        b = (sum_y - m * sum_x) / n
        return m, b
    
    def distance_to_line(self, point: tuple[float, float], m: float, b: float):
        """Calculate the perpendicular distance from point to the line y = mx + b."""
        x, y = point
        return abs(y - (m * x + b)) / math.sqrt(1 + m**2)
    
    def project_point(self, point: tuple[float, float], m: float, b: float) -> tuple[float, float]:
        """Project point onto the line y = mx + b."""
        x, y = point
        denominator = 1 + m**2
        x_proj = (x + m * y - m * b) / denominator
        y_proj = (m * x + m**2 * y + b) / denominator
        return x_proj, y_proj
    
    def find_segments(self, inliers: list[tuple[float, float]], m: float, b: float, max_gap: int, min_points=2) -> list[tuple[float, float]]:
        """Find contiguous segments of inliers along the line, excluding segments with fewer than min_points."""
        if not inliers:
            return []
        
        # Project inliers onto the line
        projections = [self.project_point(point, m, b) for point in inliers]
        
        # Sort projections along the line direction
        direction = (1, m)
        norm = math.sqrt(1 + m**2)
        direction = (1 / norm, m / norm)
        positions = [point[0] * direction[0] + point[1] * direction[1] for point in projections]
        sorted_indices = sorted(range(len(positions)), key=lambda i: positions[i])
        sorted_points = [inliers[i] for i in sorted_indices]
        sorted_positions = [positions[i] for i in sorted_indices]
        
        segments: list[tuple[float, float]] = []
        current_segment = [sorted_points[0]]
        for i in range(1, len(sorted_points)):
            if sorted_positions[i] - sorted_positions[i-1] <= max_gap:
                current_segment.append(sorted_points[i])
            else:
                if len(current_segment) >= min_points:
                    segments.append(current_segment)
                current_segment = [sorted_points[i]]
        if len(current_segment) >= min_points:
            segments.append(current_segment)
        
        return segments

    def calculate_segment_length(self, segment: tuple[float, float], m: float, b: float) -> float:
        """Calculate the length of the segment along the line."""
        if len(segment) < 2:
            return 0.0
        projections = [self.project_point(point, m, b) for point in segment]
        positions = [p[0] * (1 / math.sqrt(1 + m**2)) + p[1] * (m / math.sqrt(1 + m**2)) for p in projections]
        length = max(positions) - min(positions)
        return length

class _lidarTools:
    line = _lineCalculations()

    def getMotionLightring(self, lidar_scans: list[float], red: int = None, green: int = None, blue: int = None) -> list[LedColor]:
        """Returns a list of LEDs that are highlighted based on Lidar scans."""
        
        ledtools = _ledTools()
        if lidar_scans and min(lidar_scans) < 35:
            scans = lidar_scans
            rotation = scans.index(min(scans)) / len(scans) # Returns percentage between 0.0 to 1.
            
            if red is not None and green is not None and blue is not None:
                led = LedColor(red=red, green=green, blue=blue)
            else:
                led = ledtools.getHuePercentage(rotation)
            
            lightring = []
            for i in range(6):
                lightring += [ledtools.adjustRotationBrightness(led, rotation, _robotValues().getLedAngle(i))]
                
            return lightring
            
        return None
    
    def getCoords(self, lidar_scans: list[float], index: int, angle_increment: float, robot_position: Position) -> tuple[float, float]:
        """Returns a coordinates for an individual laser scan."""
        distance = lidar_scans[index]
        if math.isinf(distance):
            return None
        angle = angle_increment * index
        x = -distance * math.cos(angle) + robot_position.x
        y = distance * math.sin(angle) + robot_position.y
        return (x, y)
    
    def find_lines_and_segments(self, points: list[tuple[float, float]], max_iterations=100, distance_threshold=1, min_inliers=30, max_gap=5, min_points_per_segment=30) -> list[tuple[float, tuple[float, float], tuple[float, float]]]:
        """
        Find lines and their segments in a set of 2D points using RANSAC, returning x-limits for each segment.
        
        Args:
            points: List of (x, y) tuples.
            max_iterations: Maximum RANSAC iterations per line.
            distance_threshold: Max distance from line for a point to be an inlier.
            min_inliers: Minimum number of inliers to accept a line.
            max_gap: Maximum gap between points along the line to be in the same segment.
            min_points_per_segment: Minimum number of points required to form a segment.
        
        Returns:
            List of tuples: (segment length, (slope m, intercept b), (xmin, xmax))
        """
        remaining_points = points.copy()
        results = []
        
        while len(remaining_points) >= min_inliers:
            best_inliers = []
            for _ in range(max_iterations):
                p1, p2 = random.sample(remaining_points, 2)
                try:
                    m, b = self.line.fit_line([p1, p2])
                    inliers = [point for point in remaining_points if self.line.distance_to_line(point, m, b) < distance_threshold]
                    if len(inliers) > len(best_inliers):
                        best_inliers = inliers
                except ValueError:
                    continue
            
            if len(best_inliers) < min_inliers:
                break
            
            m, b = self.line.fit_line(best_inliers)
            segments = self.line.find_segments(best_inliers, m, b, max_gap, min_points=min_points_per_segment)
            
            for segment in segments:
                if len(segment) >= min_points_per_segment:
                    # Project the first and last points onto the line
                    proj_first = self.line.project_point(segment[0], m, b)
                    proj_last = self.line.project_point(segment[-1], m, b)
                    xmin = proj_first[0]
                    xmax = proj_last[0]
                    length = self.line.calculate_segment_length(segment, m, b)
                    results.append((length, (m, b), (xmin, xmax)))
            
            remaining_points = [point for point in remaining_points if point not in best_inliers]
        
        return results

class _rpiTools:
    lidar = _lidarTools()

class _pcTools:
    def getJoyTwist(self, x: float, y: float):
        twist = Twist()
        x = x * -1  # inverses X axis direction to -->
        if y:
            xc = x * math.sqrt(1 - (1/2 * y**2))  # Map square to circle and find X
            yc = y * math.sqrt(1 - (1/2 * x**2))  # Map square to circle and find Y
            # find_radius * robot_speed_in_meters * forward/reverse_+/-
            twist.linear.x = math.sqrt(xc**2 + yc**2) * (_robotValues.maxSpeed / 100) * (y / abs(y))  # m/s

        if x:
            k = (4 * _robotValues.maxSpeed) / (math.pi * _robotValues.wheelDistanceApart)  # % different
            # (find_angle - Move_by_quater_circle) * k * left/right_+/-
            twist.angular.z = (math.atan(abs(1 / x)) - (math.pi / 2)) * k * (x / abs(x))  # rad/s

        return twist

class RosTools:
    """Full list of useful tools for the Create3 robot."""
    robot = _robotTools()
    rpi = _rpiTools()
    pc = _pcTools()

    def objectTOString(obj) -> str:
        """Returns a pretty string with the object data"""

        if isinstance(obj, str):
            return obj

        if hasattr(obj, '__dict__'):
            data = vars(obj)
        else:
            data = obj

        return pprint.pformat(data, indent = 4, width = 80)