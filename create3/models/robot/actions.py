from enum import StrEnum, auto

class Actions(StrEnum):
    def _generate_next_value_(name, start, count, last_values):
        # Add a prefix to the auto-generated name
        return f"/{name.lower()}"
    
    LED_ANIMATION = auto()
    AUDIO_NOTE_SEQUENCE = auto()
    NAVIGATE_TO_POSITION = auto()
    DRIVE_ARC = auto()
    DRIVE_DISTANCE = auto()
    ROTATE_ANGLE = auto()
    DOCK = auto()
    UNDOCK = auto()