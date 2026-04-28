#
# Common Models for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from threading import Thread
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Generic, TypeVar

from rclpy.time import Time

T = TypeVar("T")

@dataclass
class QuaternionAngles:
    """Stores a quaternion orientation (x, y, z, w)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 0.0

@dataclass
class EulerAngles:
    """Stores Euler angles (roll, pitch, yaw) in radians."""

    roll_x: float = 0.0
    pitch_y: float = 0.0
    yaw_z: float = 0.0

@dataclass
class Position:
    """Stores a 2D robot position and heading.

    Units:
        x, y    → centimeters
        angle   → degrees
    """

    x: int | float = 0.0
    y: int | float = 0.0
    angle: int | float = 0.0

@dataclass
class Direction:
    """Stores the direction and distance to a target point."""

    distance: float = 0.0
    angle: float = 0.0

@dataclass
class RansacConfig:
    """Unified configuration for RANSAC and MSAC line/circle detection."""

    max_iterations: int = 0
    distance_threshold: float = 0.0      # maximum distance for a point to be considered an inlier
    min_inliers: int = 0                 # minimum number of inliers required to accept a model
    max_gap: float = 0.0                 # lines: max distance gap | circles: max angular gap (degrees)
    min_points: int = 0                  # minimum points required for a valid segment/arc
    
class Button:
    """A single button that tracks its pressed state and supports rising-edge callbacks.

    Callbacks are executed in separate daemon threads so they never block the
    main controller task or ROS callbacks.
    """

    def __init__(self) -> None:
        self._is_pressed: bool = False
        self._was_pressed: bool = False
        self._callbacks: list[Callable[[], None]] = []

    def pressed(self, callback: Callable[[], None]) -> Callable[[], None]:
        """Register a callback to be called on the rising edge (when the button is first pressed).

        The callback is executed in its own daemon thread.

        Usage:
            @controller.buttons.l1.pressed
            def on_l1_pressed():
                ...
        """
        self._callbacks.append(callback)
        return callback  # allows use as a decorator

    def __bool__(self) -> bool:
        """Allow direct boolean usage: `if ctrl.buttons.l1:`"""
        return self._is_pressed

    def _update_state(self, is_pressed: bool) -> None:
        """Update the raw button state (called internally by joy_callback)."""
        self._is_pressed = is_pressed

    def _check_and_trigger(self) -> None:
        """Check for rising edge and trigger all registered callbacks.

        Called by controller_task on every update cycle.
        """
        if self._is_pressed and not self._was_pressed:
            for callback in self._callbacks:
                # Run each callback in its own daemon thread (non-blocking)
                thread = Thread(
                    target=self._safe_call,
                    args=(callback,),
                    daemon=True,  # automatically cleaned up on shutdown
                )
                thread.start()

        self._was_pressed = self._is_pressed

    def _safe_call(self, callback: Callable[[], None]) -> None:
        """Safely execute a callback and log any exceptions."""
        try:
            callback()
        except Exception as e:  # noqa: BLE001
            print(f"Error in button callback: {e}")

@dataclass
class Stamped(Generic[T]):
    """Generic timestamped wrapper for any data type.

    Used throughout the SDK to attach a precise ROS timestamp to
    sensor readings, messages, positions, detections, or any other value.
    """

    data: T
    """The actual data payload (any type)."""

    timestamp: Time = field(default_factory=lambda: Time.from_msg(Time().to_msg()))
    """ROS clock timestamp when the data was captured or created."""
    
    # Auto-generated friendly name from the wrapped object's class
    name: str = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """Automatically set a readable name based on the wrapped data's class."""
        if hasattr(self.data, "__class__") and hasattr(self.data.__class__, "__name__"):
            self.name = self.data.__class__.__name__
        else:
            self.name = type(self.data).__name__

    def __repr__(self) -> str:
        """Clean, informative string representation."""
        return f"Stamped(data={self.data!r}, timestamp={self.timestamp})"