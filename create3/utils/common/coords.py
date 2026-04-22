#
# Coordinate Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from create3.models.common import Position, QuaternionAngles, EulerAngles, Direction

def find_direction(new_pos: tuple[float, float], current_pos: Position) -> Direction:
    """Calculate the distance between current position and new position and at which angle to turn to face it."""
    dif_x = (new_pos[0] - current_pos.x) # Difference in X coords and convert to (cm) | x(cm) - current_x(cm)
    dif_y = (new_pos[1] - current_pos.y) # Difference in Y coords and convert to (cm) | y(cm) - current_y(cm)
    
    distance = math.sqrt(dif_x**2 + dif_y**2) # Get distance to move to new coords (Pythagorean Theorem) | sqrt( difference_x(cm)^2 + difference_y(cm)^2 ) = distance(cm)
    angle = math.atan2(dif_y, dif_x) - math.radians(current_pos.angle) # Get angle (in radians) to Turn to for new coords | atan2( difference_y(cm), difference_x(cm) ) - current_angle(rad) = angle_facing_move(rad)
    
    return Direction(distance=distance, angle=angle)

def convert_to_euler(x: int | float, y: int | float, z: int | float, w: int | float) -> EulerAngles:
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
    
    return EulerAngles(roll_x=roll_x, pitch_y=pitch_y, yaw_z=yaw_z) # in radians
    
def convert_to_quaternion(roll_x: int | float, pitch_y: int | float, yaw_z: int | float) -> QuaternionAngles:
    """
    Convert a euler angle into quaternion (x, y, z, w)
    input in radians
    """

    w = round(math.cos(roll_x / 2)* math.cos(pitch_y / 2) * math.cos(yaw_z / 2) + math.sin(roll_x / 2) * math.sin(pitch_y / 2) * math.sin(yaw_z / 2), 15)
    x = round(math.sin(roll_x / 2)* math.cos(pitch_y / 2) * math.cos(yaw_z / 2) - math.cos(roll_x / 2) * math.sin(pitch_y / 2) * math.sin(yaw_z / 2), 15)
    y = round(math.cos(roll_x / 2)* math.sin(pitch_y / 2) * math.cos(yaw_z / 2) + math.sin(roll_x / 2) * math.cos(pitch_y / 2) * math.sin(yaw_z / 2), 15)
    z = round(math.cos(roll_x / 2)* math.cos(pitch_y / 2) * math.sin(yaw_z / 2) - math.sin(roll_x / 2) * math.sin(pitch_y / 2) * math.cos(yaw_z / 2), 15)
    
    return QuaternionAngles(x=x, y=y, z=z, w=w)