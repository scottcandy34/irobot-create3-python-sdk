#
# Coordinate Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
from create3.models.common import Position, QuaternionAngles, EulerAngles, Direction

def find_direction(target_pos: tuple[float, float], current_pos: Position) -> Direction:
    """Calculate straight-line distance and shortest relative heading error to a target position.

    This follows standard 2D navigation / waypoint-following math used in ROS
    and differential-drive robots.

    Parameters
    ----------
    target_pos : tuple[float, float]
        Target coordinates as (x_cm, y_cm).
    current_pos : Position
        Current robot pose containing:
        - .x (cm)
        - .y (cm)
        - .angle (degrees)

    Returns
    -------
    Direction
        Container with:
        - distance : float (cm) — Euclidean distance to target
        - angle    : float (radians) — relative turn angle, normalized to [-π, π]
          (shortest direction, never turns more than 180°)
    """
    # Difference vector in cm
    delta_x = target_pos[0] - current_pos.x
    delta_y = target_pos[1] - current_pos.y

    # Straight-line distance
    distance_cm = math.sqrt(delta_x**2 + delta_y**2)

    if distance_cm < 1e-4:  # Already at target
        return Direction(distance=0.0, angle=0.0)

    # Absolute heading toward target (world frame)
    target_heading_rad = math.atan2(delta_y, delta_x)

    # Current heading in radians
    current_heading_rad = math.radians(current_pos.angle)

    # Relative angle to turn
    angle_to_turn = target_heading_rad - current_heading_rad

    # Normalize to shortest turn: [-π, π]
    angle_to_turn = (angle_to_turn + math.pi) % (2 * math.pi) - math.pi

    return Direction(distance=distance_cm, angle=angle_to_turn)

def convert_to_euler(x: float, y: float, z: float, w: float) -> EulerAngles:
    """Convert a quaternion (x, y, z, w) into Euler angles (roll, pitch, yaw) in radians.

    This uses the standard ZYX (yaw-pitch-roll) convention common in ROS,
    robotics, and computer vision.

    Parameters
    ----------
    x : float
        Quaternion x component.
    y : float
        Quaternion y component.
    z : float
        Quaternion z component.
    w : float
        Quaternion w (scalar) component.

    Returns
    -------
    EulerAngles
        Container with:
        - roll_x  : float (rad) — rotation around X axis
        - pitch_y : float (rad) — rotation around Y axis
        - yaw_z   : float (rad) — rotation around Z axis
    """
    # Roll (rotation around x-axis)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    # Pitch (rotation around y-axis) — clamped to avoid domain error
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # Yaw (rotation around z-axis)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return EulerAngles(roll_x=roll_x, pitch_y=pitch_y, yaw_z=yaw_z)

def convert_to_quaternion(roll_x: float, pitch_y: float, yaw_z: float) -> QuaternionAngles:
    """Convert Euler angles (roll, pitch, yaw) in radians into a quaternion (x, y, z, w).

    This produces a normalized quaternion using the standard ZYX convention.
    Values are rounded to 15 decimal places to clean up floating-point noise
    (matching the behavior of your original function).

    Parameters
    ----------
    roll_x : float
        Roll angle in radians (rotation around X).
    pitch_y : float
        Pitch angle in radians (rotation around Y).
    yaw_z : float
        Yaw angle in radians (rotation around Z).

    Returns
    -------
    QuaternionAngles
        Container with quaternion components (x, y, z, w).
    """
    # Half-angle values
    cy = math.cos(yaw_z * 0.5)
    sy = math.sin(yaw_z * 0.5)
    cp = math.cos(pitch_y * 0.5)
    sp = math.sin(pitch_y * 0.5)
    cr = math.cos(roll_x * 0.5)
    sr = math.sin(roll_x * 0.5)

    # Standard quaternion conversion (ZYX)
    w = round(cr * cp * cy + sr * sp * sy, 15)
    x = round(sr * cp * cy - cr * sp * sy, 15)
    y = round(cr * sp * cy + sr * cp * sy, 15)
    z = round(cr * cp * sy - sr * sp * cy, 15)

    return QuaternionAngles(x=x, y=y, z=z, w=w)