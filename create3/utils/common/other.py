#
# Other Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import pprint
from typing import Any

TIMEOUT = 0.8 # timeout for action servers
DEFAULT_WAIT = 3 # delay in receiving command

def object_to_string(obj: Any) -> str:
    """Convert any object into a nicely formatted string for logging and debugging.

    Behavior:
      - If the input is already a string, it is returned unchanged.
      - If the object has a `__dict__` (most custom classes, dataclasses, etc.),
        its attributes are extracted via `vars()` and pretty-printed.
      - Everything else is passed directly to the pretty-printer.

    This helper is used by the `Logger` class (and anywhere else you want
    readable output of complex objects like states, messages, configs, etc.).
    """
    if isinstance(obj, str):
        return obj

    # Extract attributes if this is a custom class/object
    if hasattr(obj, "__dict__"):
        data = vars(obj)
    else:
        data = obj

    return pprint.pformat(data, indent=4, width=80)