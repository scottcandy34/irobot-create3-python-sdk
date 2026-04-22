import math

from geometry_msgs.msg import Twist

from .constraints import WHEEL_DISTANCE_APART

def get_twist(speed: float, radius: float) -> Twist:
    """Returns a twist calculated from speed and radius in cm/s and cm."""
    twist = Twist()
    twist.angular.z = speed / radius if radius else 0.0 # speed(cm/s) / radius(cm) = angular_velocity(rad/s)
    twist.linear.x = speed / 100 if radius != (WHEEL_DISTANCE_APART / 2) else 0.0 # convert to meters
    return twist

def get_time_with_distance(twist: Twist, distance: float = 0.0) -> float:
    """Returns time for how long it takes to complete a move."""
    return (distance / 100) / twist.linear.x # time = distance(m) / linear_velocity(m/s)
    
def get_time_with_angle(twist: Twist, angle: float) -> float:
    """Returns time for how long it takes to complete a turn."""
    return math.radians(angle) / twist.angular.z # time = angle(rad) / angular_velocity(rad/s)
