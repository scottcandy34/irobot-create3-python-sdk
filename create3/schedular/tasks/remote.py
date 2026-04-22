from torch import TYPE_CHECKING

from std_msgs.msg import Float32

from create3.models import Nodes
from create3.models.remote import Tasks
from create3 import RobotNode, CompanionNode, RemoteNode

if TYPE_CHECKING:
    from create3.schedular import TaskSchedular

def controller_task(scheduler: "TaskSchedular") -> None:
    """Handle remote controller input and translate it into robot actions.

    Controls:
      • R1 + left joystick   → arcade drive (forward/back + turn)
      • Options button       → dock / undock (toggles based on current state)
      • L1 + right joystick  → servo pan (vertical axis) + drive (horizontal axis)
    """
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)
    remote: RemoteNode = scheduler._get_device(Nodes.CREATE3_REMOTE)
    companion: CompanionNode | None = scheduler._get_device(Nodes.CREATE3_COMPANION)

    ctrl = remote.get_controller()

    # === Drive mode (R1 held) ===
    if ctrl.buttons.r1:
        twist_msg = remote.tools.joy.get_twist(ctrl.left_joy.horizontal, ctrl.left_joy.vertical)
        robot.send_twist(twist_msg)
        return

    # === Docking / Undocking (Options button) ===
    docking = robot.get_docking_values()

    if ctrl.buttons.options and docking.is_docked:
        scheduler.print("Undocking")
        robot.dock()
        scheduler.print("Undocking Completed")

    elif ctrl.buttons.options and not docking.is_docked:
        scheduler.print("Docking")
        robot.undock()
        scheduler.print("Docking Completed")

    # === Servo pan + drive mode (L1 held) ===
    elif ctrl.buttons.l1 and companion is not None:
        # Right stick vertical → servo angle (45° to 135° range)
        servo_msg = Float32()
        servo_msg.data = ((ctrl.right_joy.vertical + 1.0) / 2.0 * 90.0) + 45.0
        companion._servo.publish(servo_msg)

        # Right stick horizontal → pure turning
        twist_msg = remote.tools.joy.get_twist(ctrl.right_joy.horizontal, 0.0)
        robot.send_twist(twist_msg)