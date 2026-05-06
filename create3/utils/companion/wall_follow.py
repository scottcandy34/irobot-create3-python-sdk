#
# Wall Following Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import numpy as np

from geometry_msgs.msg import Twist

from create3.models.companion import Lidar
from create3.utils.common import PID
from create3.utils.robot.constraints import RADIUS, MAX_SPEED, MAX_ANGULAR_SPEED

pid_angular = PID(kp = 0.04, ki = 0.0, kd = 0.0, reference = RADIUS + 10, output_min = -MAX_ANGULAR_SPEED, output_max = MAX_ANGULAR_SPEED)
pid_linear = PID(kp = 1.5, ki = 0.01, kd = 0.8, reference = RADIUS + 10, output_min = 0, output_max = MAX_SPEED)

def pid_lidar_to_twist(lidar: Lidar) -> Twist:
    """Convert LiDAR scan data into a Twist command for simple right-hand wall following using PID control.

    Behavior:
      • Uses the right-side slice to maintain a desired wall distance (pid_angular)
      • Uses the front slice to slow down or stop when an obstacle is ahead (pid_linear)
      • Filters out invalid readings (too close or beyond sensor max range)
      • Linear speed is returned in m/s (cm/s values from PID are divided by 100)

    This is a classic reactive wall-follower commonly used in introductory ROS robots.

    Parameters
    ----------
    lidar : Lidar
        LiDAR sensor object providing `get_right_slice()` and `get_front_slice()`
        (both return lists of distance measurements in cm).

    Returns
    -------
    Twist
        cmd_vel message with:
        - linear.x : forward speed in m/s
        - angular.z : turning rate in rad/s (positive = left turn)
    """
    range_max = lidar.range_max
    min_valid = 0.1

    # Convert slices (still cheap even if input is Python list)
    right = np.asarray(lidar.get_right_slice(), dtype=np.float32)
    front = np.asarray(lidar.get_front_slice(), dtype=np.float32)

    # ── Right wall distance (single mask, no copy) ──
    valid_right = (right > min_valid) & (right < range_max)
    wall_dist = np.mean(right, where=valid_right)
    if np.isnan(wall_dist):          # all values invalid → fallback
        wall_dist = range_max

    # ── Front obstacle distance (same pattern) ──
    valid_front = (front > min_valid) & (front < range_max)
    front_dist = np.mean(front, where=valid_front)
    if np.isnan(front_dist):
        front_dist = range_max

    # Run PID controllers
    angular_output = pid_angular(wall_dist)
    linear_output = pid_linear(front_dist)

    # Build Twist message
    twist_msg = Twist()
    twist_msg.angular.z = angular_output
    twist_msg.linear.x = linear_output / 100.0   # cm/s → m/s
    return twist_msg