#
# Interrupt for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import signal

from .ros_threading import Threading

class Interrupt():
    def __init__(self):
        self._devices: list[Threading] = []

        signal.signal(signal.SIGINT, self._sigint_handler)

    def add_device(self, device: Threading):
        """Add a device to the interrupt to watch for CTL-C."""
        self._devices.append(device)

    def _sigint_handler(self, sig, frame):
        for device in self._devices:
            device.shutdown()
        raise SystemExit("SIGINT received-shut down complete.")
    
global_interrupt = Interrupt()
