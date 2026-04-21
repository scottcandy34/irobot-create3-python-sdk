#
# Wall Following Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from geometry_msgs.msg import Twist

from create3.models.companion import Lidar
from create3.utils.common import PID
from create3.utils.robot.constraints import RADIUS, MAX_SPEED, MAX_ANGULAR_SPEED

pid_angular = PID(kp = 1.5, ki = 0.01, kd = 0.8, setpoint = RADIUS + 10, output_min = -MAX_ANGULAR_SPEED, output_max = MAX_ANGULAR_SPEED)
pid_linear = PID(kp = 1.5, ki = 0.01, kd = 0.8, setpoint = RADIUS + 10, output_min = 0, output_max = MAX_SPEED)

def pid_lidar_to_twist(lidar: Lidar) -> Twist:
    """Simple wall following behavior using PID control based on lidar data."""
    left_distances = [d  for d in lidar.get_left_slice() if 0.1 < d < lidar.range_max]
    front_distances = [d for d in lidar.get_front_slice() if 0.1 < d < lidar.range_max]

    wall_dist = sum(left_distances) / len(left_distances) / 100
    front_dist = sum(front_distances) / len(front_distances) / 100

    angular_output = pid_angular(wall_dist)
    linear_output = pid_linear(front_dist)

    twist_msg = Twist()
    twist_msg.angular.z = angular_output
    twist_msg.linear.x = linear_output

    return twist_msg