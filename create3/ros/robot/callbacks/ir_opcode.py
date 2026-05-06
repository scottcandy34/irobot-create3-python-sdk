#
# Message Callback Functions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from typing import TYPE_CHECKING

from irobot_create_msgs.msg import IrOpcode

if TYPE_CHECKING:
    from create3.ros.robot import Subscriber

def ir_opcode_callback(subscriber: "Subscriber", ir_opcode: IrOpcode) -> None:
    """Parse IR docking opcodes and set the corresponding buoy/force-field flags."""
    subscriber.docking_values.sensor = ir_opcode.sensor

    match ir_opcode.opcode:
        case 161:
            subscriber.docking_values.redBuoy = False
            subscriber.docking_values.greenBuoy = False
            subscriber.docking_values.forceField = True
        case 164:
            subscriber.docking_values.redBuoy = False
            subscriber.docking_values.greenBuoy = True
            subscriber.docking_values.forceField = False
        case 165:
            subscriber.docking_values.redBuoy = False
            subscriber.docking_values.greenBuoy = True
            subscriber.docking_values.forceField = True
        case 168:
            subscriber.docking_values.redBuoy = True
            subscriber.docking_values.greenBuoy = False
            subscriber.docking_values.forceField = False
        case 169:
            subscriber.docking_values.redBuoy = True
            subscriber.docking_values.greenBuoy = False
            subscriber.docking_values.forceField = True
        case 172:
            subscriber.docking_values.redBuoy = True
            subscriber.docking_values.greenBuoy = True
            subscriber.docking_values.forceField = False
        case 173:
            subscriber.docking_values.redBuoy = True
            subscriber.docking_values.greenBuoy = True
            subscriber.docking_values.forceField = True
        