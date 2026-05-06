from enum import StrEnum, auto

class Actions(StrEnum):
    LED_ANIMATION = auto()
    AUDIO_NOTE_SEQUENCE = auto()
    NAVIGATE_TO_POSITION = auto()
    DRIVE_ARC = auto()
    DRIVE_DISTANCE = auto()
    ROTATE_ANGLE = auto()
    DOCK = auto()
    UNDOCK = auto()