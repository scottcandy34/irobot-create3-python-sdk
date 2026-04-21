from torch import TYPE_CHECKING

from std_msgs.msg import Float32

from create3.models import Nodes
from create3.models.remote import Tasks
from create3 import RobotNode, CompanionNode, RemoteNode

if TYPE_CHECKING:
    from create3.schedular import TaskSchedular

def controller_task(scheduler: "TaskSchedular"):
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)
    remote: RemoteNode = scheduler._get_device(Nodes.CREATE3_REMOTE)
    companion: CompanionNode = scheduler._get_device(Nodes.CREATE3_COMPANION)  # optional

    ctrl = remote.get_controller()

    if ctrl.buttons.r1:
        twist_msg = remote.tools.joy.get_twist(ctrl.left_joy.horizontal, ctrl.left_joy.vertical)
        robot.send_twist(twist_msg)

    elif ctrl.buttons.options and robot.get_docking_values().is_docked:
        scheduler.print('Undocking')
        robot.dock()
        scheduler.print('Undocking Completed')

    elif ctrl.buttons.options and not robot.get_docking_values().is_docked:
        scheduler.print('Docking')
        robot.undock()
        scheduler.print('Docking Completed')

    elif ctrl.buttons.l1:
        servo_msg = Float32()
        servo_msg.data = ((ctrl.right_joy.vertical + 1)/ 2 * 90) + 45
        companion._servo.publish(servo_msg)

        twist_msg = remote.tools.joy.get_twist(ctrl.right_joy.horizontal, 0.0)
        robot.send_twist(twist_msg)