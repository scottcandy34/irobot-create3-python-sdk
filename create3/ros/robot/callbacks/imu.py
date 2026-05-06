#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from rclpy.time import Time
from sensor_msgs.msg import Imu

from create3.models.common import Stamped
from create3.models.robot import Acceleration

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def imu_callback(subscriber: "Subscriber", imu: Imu) -> None:
    """Extract linear acceleration from the IMU and store it in the shared state."""
    acceleration = Acceleration()

    accel = imu.linear_acceleration
    acceleration.x = accel.x
    acceleration.y = accel.y
    acceleration.z = accel.z
    
    subscriber.acceleration = Stamped(acceleration, Time.from_msg(imu.header.stamp))
