import math

from geometry_msgs.msg import Twist

from .constraints import WHEEL_DISTANCE_APART

def get_twist(speed_cm_s: float, radius_cm: float) -> Twist:
    """Create a ROS Twist from speed and turning radius (standard differential-drive way).

    This follows the conventional kinematics used in ROS navigation, teleop,
    and most differential-drive robots (TurtleBot, etc.).

    Parameters
    ----------
    speed_cm_s : float
        Speed in cm/s.
        • radius_cm != 0 → linear speed of the robot's center
        • radius_cm == 0   → tangential speed of each wheel (pure rotation)
    radius_cm : float
        Turning radius in cm measured from the robot's center.
        Special value: 0.0 = pure spin-in-place (rotate on the spot).
        Negative radius reverses the turn direction.

    Returns
    -------
    Twist
        Ready-to-publish cmd_vel message:
        - linear.x  in m/s
        - angular.z in rad/s
    """
    twist = Twist()
    d = WHEEL_DISTANCE_APART  # track width / wheel separation in cm

    if abs(radius_cm) < 1e-4:  # Pure rotation in place
        twist.linear.x = 0.0
        twist.angular.z = 2.0 * speed_cm_s / d   # ω = 2·v_wheel / d

    else:  # Normal arc motion
        twist.linear.x = speed_cm_s / 100.0          # cm/s → m/s
        twist.angular.z = speed_cm_s / radius_cm     # ω = v / r  (rad/s)
    
    return twist

def get_motion_time(twist: Twist, distance_cm: float = 0.0, angle_deg: float = 0.0) -> float:
    """Calculate the time (in seconds) needed to complete a motion command.

    Handles translation, rotation, or both simultaneously (common for arc motion).
    Returns the *maximum* of the two times — this is the conservative/safe choice
    used in most differential-drive controllers so the slower motion finishes first.

    Parameters
    ----------
    twist : Twist
        Velocity command containing linear.x (m/s) and/or angular.z (rad/s).
    distance_cm : float
        Straight-line distance to travel (cm). 0 = no translation.
    angle_deg : float
        Rotation angle (degrees). 0 = no rotation.
        Positive/negative determines direction, but time is always positive.

    Returns
    -------
    float
        Time in seconds. Returns 0.0 if the robot has no motion (both linear and
        angular velocities are zero).
    """
    # Time needed for translation
    if abs(twist.linear.x) < 1e-6 or distance_cm <= 0.0:
        time_distance = 0.0
    else:
        time_distance = (distance_cm / 100.0) / abs(twist.linear.x)

    # Time needed for rotation
    if abs(twist.angular.z) < 1e-6 or angle_deg == 0.0:
        time_angle = 0.0
    else:
        time_angle = math.radians(abs(angle_deg)) / abs(twist.angular.z)

    # Return the longer of the two (conservative for simultaneous motion)
    return max(time_distance, time_angle)