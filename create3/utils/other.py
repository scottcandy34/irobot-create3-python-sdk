#
# Other Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import pprint as _pprint

TIMEOUT = 0.8 # timeout for action servers
DEFAULT_WAIT = 3 # delay in receiving command

def object_to_string(obj) -> str:
    """Returns a pretty string with the object data"""

    if isinstance(obj, str):
        return obj

    if hasattr(obj, '__dict__'):
        data = vars(obj)
    else:
        data = obj

    return _pprint.pformat(data, indent = 4, width = 80)