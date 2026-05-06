#
# Handler Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist

if TYPE_CHECKING:
    from create3.ros.robot import Publisher

def set_velocity_handler_callback(publisher: "Publisher") -> None:
    """Periodically publish wheel velocity commands (/cmd_vel) every 0.05 seconds.

    This handler ensures continuous robot motion by:
      • Publishing immediately when the wheel speed command changes
      • Continuously re-publishing any non-empty (non-zero) command every cycle
        (standard ROS practice to prevent velocity timeout/safety stop)

    Zero commands (default Twist()) are not re-published.
    """
    current = publisher.velocity

    # Publish on change OR if we have an active non-zero movement command
    if (current != publisher.last_velocity) or (current != Twist()):
        publisher.send_velocity(current)

    # Always update the last-known command for next comparison
    publisher.last_velocity = current
