from enum import StrEnum, auto

class Services(StrEnum):
    def _generate_next_value_(name, start, count, last_values):
        # Add a prefix to the auto-generated name
        return f"/{name.lower()}"
        
    RESET_POSE = auto()