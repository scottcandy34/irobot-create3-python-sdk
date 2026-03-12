#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from .objects import Controller

class Subscribe():
    """Holds all remote subscribed topics."""
    controller = Controller()

class Publish():
    """Holds all remote published topics."""
    rumble_enable: bool = False