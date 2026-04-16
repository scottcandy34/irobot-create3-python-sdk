#
# Topic Definitions for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from .objects import Controller, Map

class Subscribe():
    """Holds all remote subscribed topics."""
    controller = Controller()
    map = Map()

class Publish():
    """Holds all remote published topics."""
    rumble_enable: bool = False