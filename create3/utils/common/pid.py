#
# PID Controller Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import time

class PID:
    """A PID controller implementation with anti-windup and derivative filtering."""
    def __init__(self, kp: float, ki: float, kd: float, reference: float = 0.0, derivative_tau: float = 0.08, output_min: float = -float('inf'), output_max: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reference = reference
        self.derivative_tau = derivative_tau
        self.output_min = output_min
        self.output_max = output_max

        # state
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._prev_time = None
        self._first_call = True

        self._pid: tuple[float, float, float] = (0.0, 0.0, 0.0) # (P, I, D)
        self._error: float = 0.0
        self._output: float = 0.0

    def get_pid(self) -> tuple[float, float, float]:
        """Return the current PID (P, I, D). For use with PIDTuner."""
        return self._pid
    
    def get_error(self) -> float:
        """Returns the error correction. For use with PIDTuner."""
        return self._error
    
    def get_output(self) -> float:
        """Returns the correction. For use with PIDTuner."""
        return self._output

    def update(self, measurement: float, dt: float = None) -> float:
        """Convenience method to compute PID output. Can be used as an alias for __call__."""
        return self.__call__(measurement, dt)

    def __call__(self, measurement: float, dt: float = None) -> float:
        """
        Compute the PID output given a measurement and optional time step.
        If dt is not provided, it will be calculated based on the time since the last call.
        """
        current_time = time.perf_counter()

        # calculate dt (auto or manual)
        if dt is None:
            if self._prev_time is not None:
                dt = current_time - self._prev_time
            else:
                dt = 0.1 # safe fallback on first call
        if dt <= 0.0:
            dt = 0.1

        error = self.reference - measurement

        # Proportional
        P = self.kp * error

        # Integral with anti-windup (will be adjusted after output clamping)
        self._integral += error * dt
        I = self.ki * self._integral

        # Derivative (use delta_time if provided for real units)
        if self._first_call:
            raw_deriv = 0.0
            self._first_call = False
        else:
            raw_deriv = (error - self._prev_error) / dt

        alpha = self.derivative_tau / (self.derivative_tau + dt)
        filtered_deriv = alpha * self._prev_derivative + (1 - alpha) * raw_deriv

        D = self.kd * filtered_deriv

        # Raw PID sum
        output = P + I + D

        # Clamp output and apply anti-windup
        if output > self.output_max:
            output = self.output_max
            # Anti-windup: prevent integral from increasing if output is saturated
            if self.ki != 0:
                self._integral = (output - P - D) / self.ki
        elif output < self.output_min:
            output = self.output_min
            # Anti-windup: prevent integral from decreasing if output is saturated
            if self.ki != 0:
                self._integral = (output - P - D) / self.ki

        # Update state
        self._prev_error = error
        self._prev_derivative = filtered_deriv
        self._prev_time = current_time

        # save final values for use in pid tuner
        self._pid = (P, I, D)
        self._error = error
        self._output = output

        return output

    def set_reference(self, reference: float):
        """Update the reference for the PID controller."""
        self.reference = reference

    def set_output_limits(self, min_out: float, max_out: float):
        """Set output limits for the PID controller."""
        self.output_min = min_out
        self.output_max = max_out

    def reset(self):
        """Call when robot stops or you change mode"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._prev_time = None
        self._first_call = True