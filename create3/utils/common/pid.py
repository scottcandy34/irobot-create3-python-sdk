#
# PID Controller Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time

class PID:
    """A robust PID controller with derivative filtering and anti-windup.

    Features:
      • First-order low-pass filter on the derivative term (prevents derivative kick)
      • Anti-windup that clamps the integral term when the output saturates
      • Automatic or manual dt (uses wall-clock time if not provided)
      • Tunable output limits
      • Easy access to P, I, D, error, and output for debugging/tuning

    This is the standard form used in most ROS differential-drive controllers.
    """

    def __init__(self, kp: float, ki: float, kd: float, reference: float = 0.0, derivative_tau: float = 0.08, output_min: float = -float("inf"), output_max: float = float("inf")) -> None:
        """Initialize a PID controller.

        Parameters
        ----------
        kp, ki, kd : float
            Proportional, integral, and derivative gains.
        reference : float
            Target value (setpoint).
        derivative_tau : float
            Time constant (seconds) for the derivative low-pass filter.
            Smaller = faster response, larger = smoother.
        output_min, output_max : float
            Saturation limits for the controller output.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reference = reference
        self.derivative_tau = derivative_tau
        self.output_min = output_min
        self.output_max = output_max

        # Internal state
        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._prev_derivative: float = 0.0
        self._prev_time: float | None = None
        self._first_call: bool = True

        # Values exposed for PIDTuner / debugging
        self._pid: tuple[float, float, float] = (0.0, 0.0, 0.0)  # (P, I, D)
        self._error: float = 0.0
        self._output: float = 0.0

    def get_pid(self) -> tuple[float, float, float]:
        """Return the current PID components (P, I, D) for tuning/debugging."""
        return self._pid

    def get_error(self) -> float:
        """Return the current error (reference - measurement)."""
        return self._error

    def get_output(self) -> float:
        """Return the most recent controller output."""
        return self._output

    def update(self, measurement: float, dt: float | None = None) -> float:
        """Convenience alias for __call__ (identical behavior)."""
        return self.__call__(measurement, dt)

    def __call__(self, measurement: float, dt: float | None = None) -> float:
        """Compute PID output for the current measurement.

        If `dt` is not supplied, it is automatically calculated from
        wall-clock time (time.perf_counter).

        Parameters
        ----------
        measurement : float
            Current sensor reading.
        dt : float | None
            Time step in seconds. If None, computed automatically.

        Returns
        -------
        float
            Saturated PID output (P + I + D).
        """
        current_time = time.perf_counter()

        # Calculate dt (auto or manual)
        if dt is None:
            if self._prev_time is not None:
                dt = current_time - self._prev_time
            else:
                dt = 0.1  # safe fallback on first call
        if dt <= 0.0:
            dt = 0.1

        error = self.reference - measurement

        # Proportional term
        P = self.kp * error

        # Integral term (will be corrected by anti-windup later)
        self._integral += error * dt
        I = self.ki * self._integral

        # Derivative term with low-pass filter
        if self._first_call:
            raw_deriv = 0.0
            self._first_call = False
        else:
            raw_deriv = (error - self._prev_error) / dt

        alpha = self.derivative_tau / (self.derivative_tau + dt)
        filtered_deriv = alpha * self._prev_derivative + (1.0 - alpha) * raw_deriv
        D = self.kd * filtered_deriv

        # Raw output
        output = P + I + D

        # Clamp output and apply anti-windup
        if output > self.output_max:
            output = self.output_max
            if self.ki != 0.0:
                self._integral = (output - P - D) / self.ki
        elif output < self.output_min:
            output = self.output_min
            if self.ki != 0.0:
                self._integral = (output - P - D) / self.ki

        # Update internal state
        self._prev_error = error
        self._prev_derivative = filtered_deriv
        self._prev_time = current_time

        # Store final values for PID tuner / debugging
        self._pid = (P, I, D)
        self._error = error
        self._output = output

        return output

    def set_reference(self, reference: float) -> None:
        """Update the controller setpoint (reference value)."""
        self.reference = reference

    def set_output_limits(self, min_out: float, max_out: float) -> None:
        """Set new output saturation limits."""
        self.output_min = min_out
        self.output_max = max_out

    def reset(self) -> None:
        """Reset all internal state (call when robot stops or mode changes)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._prev_time = None
        self._first_call = True