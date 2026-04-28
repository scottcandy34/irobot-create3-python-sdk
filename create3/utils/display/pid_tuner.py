# =====================================================================
# iRobot Create3 - Jazzy PID Tuner (Live Tuning GUI)
# Created by scottcandy34 – heavily extended and commented for you
#
# HOW TO TUNE STEP-BY-STEP (do this while the robot is running)
# ================================================================
# 1. Start with pure P control
#    - Set ki=0 and kd=0 temporarily (use keys 2 and 3 to zero them).
#    - Slowly increase kp from ~0.3 until the robot starts to follow
#      the wall but just begins to oscillate a little.
#    - Then back it off 20–30% (that’s usually your final Kp).
#
# 2. Add Kd (damping)
#    - Increase kd until the zig-zagging calms down and the motion
#      feels smooth.
#    - (You’re already seeing why Kd is important — it counters the
#      rapid changes.)
#
# 3. Add a tiny bit of Ki (only at the end)
#    - Once the robot is mostly stable, raise ki very slowly
#      (0.005 → 0.02 max).
#    - This eliminates the slow drift away from the 50 cm line
#      over long distances.
#
# QUICK CHECKLIST while watching the robot
# ========================================
# Symptom                      | What to change
# -----------------------------|---------------
# Still oscillates / zig-zags  | Lower kp or raise kd
# Turns too slowly / drifts    | Raise kp a little
# Feels "mushy" or unresponsive| Lower kd
# Slowly creeps away from wall | Raise ki very slightly
# Turns way too sharp (±2.8)   | Lower output_max in your control loop
#
# =====================================================================

import tkinter as tk
from tkinter import simpledialog
from typing import Any

from create3.utils.common import PID

class PIDTuner:
    """Live PID tuner GUI for the iRobot Create3.

    Provides a real-time, keyboard-driven interface to adjust Kp, Ki, and Kd
    while the robot is running. The display refreshes every 50 ms so you can
    instantly see the effect on error, P/I/D terms, and controller output.

    Controls:
      • 1 / 2 / 3     → select Kp / Ki / Kd
      • ↑ / ↓          → increase / decrease by current step size
      • Tab            → cycle step size (finer ↔ coarser)
      • Enter          → type an exact value
      • Esc            → close tuner and print final tuned gains
    """

    def __init__(self, pid: PID) -> None:
        """Launch the PID tuner window attached to a live PID controller.

        Parameters
        ----------
        pid : PID
            The PID instance to tune (shares the same object, so changes
            are immediately visible to the robot control loop).
        """
        self.pid = pid

        self._selected = 0                     # 0=Kp, 1=Ki, 2=Kd
        self._param_names = ["Kp", "Ki", "Kd"]

        # Step sizes (from very fine to coarse)
        self._step_options = [0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 1.0]
        self._current_step_idx = 3             # default = 0.001

        # Create GUI
        self._root = tk.Tk()
        self._root.title("iRobot Create3 - Jazzy PID Tuner")
        self._root.resizable(False, False)

        # Large, clean, monospace display
        self._label = tk.Label(
            self._root,
            text="",
            font=("Consolas", 18, "bold"),
            fg="#00ff88",
            bg="#111111",
            justify=tk.LEFT,
            anchor="w",
        )
        self._label.pack(padx=50, pady=30)

        # On-screen help
        tk.Label(
            self._root,
            text=(
                "1 / 2 / 3     → Select Kp / Ki / Kd\n"
                "↑ / ↓          → ± current step\n"
                "Tab            → Cycle step size (finer ↔ coarser)\n"
                "Enter (⏎)     → Type exact value\n"
                "Esc            → Close tuner"
            ),
            font=("Helvetica", 12),
            fg="#cccccc",
            bg="#111111",
            justify=tk.LEFT,
        ).pack(pady=8)

        # Quit button
        quit_btn = tk.Button(
            self._root,
            text="QUIT TUNER",
            font=("Helvetica", 14, "bold"),
            bg="#ff3333",
            fg="white",
            command=self._close_gui,
            width=20,
            height=2,
        )
        quit_btn.pack(pady=12)

        # Keyboard bindings (all run safely in the GUI thread)
        self._root.bind("<Key-1>", lambda e: self._select_param(0))
        self._root.bind("<Key-2>", lambda e: self._select_param(1))
        self._root.bind("<Key-3>", lambda e: self._select_param(2))
        self._root.bind("<Up>", self._increase)
        self._root.bind("<Down>", self._decrease)
        self._root.bind("<Tab>", self._cycle_step_size)
        self._root.bind("<Return>", self._manual_edit)
        self._root.bind("<Escape>", lambda e: self._close_gui())

        # Window close button (X) also triggers clean shutdown
        self._root.protocol("WM_DELETE_WINDOW", self._close_gui)

        # Keep window on top while tuning
        self._root.attributes("-topmost", True)

        # Start real-time refresh (20 Hz)
        self._update_period_ms = 50
        self._schedule_update()

        self._root.mainloop()

    def _get_display_text(self) -> str:
        """Build the live multi-line display (called frequently)."""
        selected = [" ", " ", " "]
        selected[self._selected] = "→"

        current_step = self._step_options[self._current_step_idx]

        return (
            f"Reference:   {self.pid.reference:6.1f} cm\n"
            f"Error:       {self.pid.get_error():6.2f} cm\n\n"
            f"{selected[0]} Kp  = {self.pid.kp:8.6f}\n"
            f"{selected[1]} Ki  = {self.pid.ki:8.6f}\n"
            f"{selected[2]} Kd  = {self.pid.kd:8.6f}\n\n"
            f"Step size:   {current_step:.6f}\n"
            f"P term:      {self.pid.get_pid()[0]:8.4f}\n"
            f"I term:      {self.pid.get_pid()[1]:8.4f}\n"
            f"D term:      {self.pid.get_pid()[2]:8.4f}\n"
            f"────────────────────\n"
            f"PID Output:  {self.pid.get_output():8.4f}"
        )

    def _update_label(self) -> None:
        """Refresh the displayed text safely."""
        if self._label and self._root:
            try:
                self._label.config(text=self._get_display_text())
            except Exception:
                pass  # GUI may be closing

    def _schedule_update(self) -> None:
        """Continuously refresh the display every 50 ms."""
        self._update_label()
        if self._root:
            self._root.after(self._update_period_ms, self._schedule_update)

    def _select_param(self, index: int) -> None:
        """Switch which gain (Kp/Ki/Kd) is currently selected."""
        self._selected = index % 3
        self._update_label()

    def _get_current_step(self) -> float:
        """Return the currently selected step size."""
        return self._step_options[self._current_step_idx]

    def _cycle_step_size(self, event: Any = None) -> None:
        """Tab cycles through step sizes (finer ↔ coarser)."""
        self._current_step_idx = (self._current_step_idx + 1) % len(self._step_options)
        self._update_label()

    def _change_value(self, delta: float) -> None:
        """Add/subtract the current step from the selected gain."""
        step = self._get_current_step() if delta > 0 else -self._get_current_step()

        if self._selected == 0:      # Kp
            self.pid.kp = max(0.0, self.pid.kp + step)
        elif self._selected == 1:    # Ki
            self.pid.ki = max(0.0, self.pid.ki + step)
        else:                        # Kd
            self.pid.kd = max(0.0, self.pid.kd + step)

        self._update_label()

    def _increase(self, event: Any = None) -> None:
        """Up arrow – increase selected gain."""
        self._change_value(self._get_current_step())

    def _decrease(self, event: Any = None) -> None:
        """Down arrow – decrease selected gain."""
        self._change_value(-self._get_current_step())

    def _manual_edit(self, event: Any = None) -> None:
        """Enter key – open dialog to type an exact value."""
        param_name = self._param_names[self._selected]
        current = getattr(self.pid, self._param_names[self._selected].lower())

        new_val = simpledialog.askfloat(
            title="Manual PID Value",
            prompt=f"Enter new {param_name} value:",
            initialvalue=current,
            minvalue=0.0,
        )

        if new_val is not None:          # user clicked OK
            new_val = max(0.0, new_val)
            if self._selected == 0:
                self.pid.kp = new_val
            elif self._selected == 1:
                self.pid.ki = new_val
            else:
                self.pid.kd = new_val
            self._update_label()

    def _print_final_summary(self) -> None:
        """Print a clean, copy-paste-friendly summary when the tuner closes."""
        print("\n" + "═" * 70)
        print(" " * 22 + "🎯 PID TUNING SESSION COMPLETE 🎯")
        print("═" * 70)
        print(f"{'Final Tuned Gains':^70}")
        print("─" * 70)
        print(f"   Kp  =  {self.pid.kp:8.6f}     ← Proportional")
        print(f"   Ki  =  {self.pid.ki:8.6f}     ← Integral")
        print(f"   Kd  =  {self.pid.kd:8.6f}     ← Derivative")
        print("─" * 70)
        print("✅ Copy these values into your robot code for perfect wall following!")
        print("   (Recommended: hard-code them.)\n")

    def _close_gui(self) -> None:
        """Close the tuner window and print the final tuned gains."""
        if self._root:
            self._print_final_summary()
            try:
                self._root.destroy()
            except Exception:
                pass
            self._root = None