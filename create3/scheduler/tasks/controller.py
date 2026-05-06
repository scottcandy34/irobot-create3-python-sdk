from typing import TYPE_CHECKING

from std_msgs.msg import Float32

from create3.models.common import Nodes
from create3 import RobotNode, CompanionNode, RemoteNode
from create3.models.common import Button

if TYPE_CHECKING:
    from create3.scheduler import TaskScheduler

def controller_task(scheduler: "TaskScheduler") -> None:
    """Handle remote controller input and translate it into robot actions.

    Features:
      • Rising-edge button callbacks (via the new Button class)
      • R1 + left joystick   → arcade drive
      • L1 + right joystick  → servo pan + turning
    """
    robot: RobotNode = scheduler._get_device(Nodes.CREATE3_ROBOT)
    remote: RemoteNode = scheduler._get_device(Nodes.CREATE3_REMOTE)
    companion: CompanionNode | None = scheduler._get_device(Nodes.CREATE3_COMPANION)

    ctrl = remote.get_controller()
    
    # Check all face/shoulder/special buttons
    for attr_name in vars(ctrl.buttons):
        button = getattr(ctrl.buttons, attr_name)
        if isinstance(button, Button):
            button._check_and_trigger()
            
    # Check all D-pad buttons
    for attr_name in vars(ctrl.dpad):
        button = getattr(ctrl.dpad, attr_name)
        if isinstance(button, Button):
            button._check_and_trigger()

    # Check stick press buttons
    ctrl.left_joy.button._check_and_trigger()
    ctrl.right_joy.button._check_and_trigger()

    # === Drive mode (R1 held) ===
    if ctrl.buttons.r1:
        twist_msg = remote.tools.joy.get_twist(ctrl.left_joy.horizontal, ctrl.left_joy.vertical)
        robot.publisher.send_velocity(twist_msg)
        return

    # === Servo pan + drive mode (L1 held) ===
    if ctrl.buttons.l1 and companion is not None:
        # Right stick vertical → servo angle (45° to 135° range)
        servo_msg = Float32()
        servo_msg.data = ((ctrl.right_joy.vertical + 1.0) / 2.0 * 90.0) + 45.0
        companion.publisher.send_servo_angle(servo_msg)

        # Right stick horizontal → pure turning
        twist_msg = remote.tools.joy.get_twist(ctrl.right_joy.horizontal, 0.0)
        robot.publisher.send_velocity(twist_msg)