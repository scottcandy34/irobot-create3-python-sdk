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
from .pid import PID

class PIDTuner():
    """
    Live PID tuner GUI for iRobot Create3.
    Adjust Kp/Ki/Kd instantly while the robot runs.
    Display constantly refreshes (every 50 ms) to show fast-changing
    error, P/I/D terms, and output from the shared PID object.
    """

    def __init__(self, pid: PID):
        self.pid = pid

        self._selected = 0           # 0 = Kp, 1 = Ki, 2 = Kd
        self._param_names = ["Kp", "Ki", "Kd"]

        # Step sizes – small for fine tuning, large for quick jumps
        self._steps = { 'small': 0.001, 'large': 0.05 }

        self._root = tk.Tk()
        self._root.title("iRobot Create3 - Jazzy PID Tuner")
        self._root.resizable(False, False)

        # Big, clean, monospace display for perfect alignment
        self._label = tk.Label(
            self._root,
            text="",
            font=("Consolas", 18, "bold"),
            fg="#00ff88",
            bg="#111111",
            justify=tk.LEFT,
            anchor="w"
        )
        self._label.pack(padx=50, pady=30)

        # On-screen instructions
        tk.Label(
            self._root,
            text="1 / 2 / 3     → Select Kp / Ki / Kd\n"
                 "↑ / ↓          → ± small step (0.001)\n"
                 "Shift + ↑ / ↓  → ± large step (0.05)\n"
                 "Enter (⏎)     → Type exact value for selected\n"
                 "Esc or QUIT button → Close window",
            font=("Helvetica", 12),
            fg="#cccccc",
            bg="#111111",
            justify=tk.LEFT
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
            height=2
        )
        quit_btn.pack(pady=12)

        # Key bindings (all executed safely in the GUI thread)
        self._root.bind("<Key-1>", lambda e: self._select_param(0))
        self._root.bind("<Key-2>", lambda e: self._select_param(1))
        self._root.bind("<Key-3>", lambda e: self._select_param(2))
        self._root.bind("<Up>", self._increase)
        self._root.bind("<Down>", self._decrease)
        self._root.bind("<Shift-Up>", self._increase_large)
        self._root.bind("<Shift-Down>", self._decrease_large)
        self._root.bind("<Return>", self._manual_edit)
        self._root.bind("<Escape>", lambda e: self._close_gui())

        # Window close button (X)
        self._root.protocol("WM_DELETE_WINDOW", self._close_gui)

        # Keep window on top while testing
        self._root.attributes("-topmost", True)

        # Start continuous real-time updates (20 Hz)
        self._update_period_ms = 50
        self._schedule_update()

        self._root.mainloop()

    def _get_display_text(self) -> str:
        """Build the beautiful multi-line display (call while holding lock)"""
        # Arrow shows which parameter is currently selected
        selected = [" ", " ", " "]
        selected[self._selected] = "→"

        return (
            f"Reference:   {self.pid.reference:6.1f} cm\n"
            f"Error:       {self.pid.get_error():6.2f} cm\n\n"
            f"{selected[0]} Kp  = {self.pid.kp:8.5f}\n"
            f"{selected[1]} Ki  = {self.pid.ki:8.5f}\n"
            f"{selected[2]} Kd  = {self.pid.kd:8.5f}\n\n"
            f"P term:      {self.pid.get_pid()[0]:8.3f}\n"
            f"I term:      {self.pid.get_pid()[1]:8.3f}\n"
            f"D term:      {self.pid.get_pid()[2]:8.3f}\n"
            f"────────────────────\n"
            f"PID Output:  {self.pid.get_output():8.3f}"
        )

    def _update_label(self):
        """Refresh the GUI safely"""
        if self._label and self._root:
            try:
                self._label.config(text=self._get_display_text())
            except:
                pass

    def _schedule_update(self):
        """Continuously refresh every 50 ms so error/P/I/D/output stay live"""
        self._update_label()
        if self._root:
            self._root.after(self._update_period_ms, self._schedule_update)

    def _select_param(self, index: int):
        """Switch which gain you are adjusting right now"""
        self._selected = index % 3
        self._update_label()

    def _change_value(self, delta: float):
        """Add delta to the currently selected gain (non-negative)"""
        if self._selected == 0:      # Kp
            self.pid.kp = max(0.0, self.pid.kp + delta)
        elif self._selected == 1:    # Ki
            self.pid.ki = max(0.0, self.pid.ki + delta)
        elif self._selected == 2:    # Kd
            self.pid.kd = max(0.0, self.pid.kd + delta)
        self._update_label()

    def _increase(self, event=None):
        self._change_value(self._steps['small'])

    def _decrease(self, event=None):
        self._change_value(-self._steps['small'])

    def _increase_large(self, event=None):
        self._change_value(self._steps['large'])

    def _decrease_large(self, event=None):
        self._change_value(-self._steps['large'])
        
    def _manual_edit(self, event=None):
        """NEW: Open a dialog so you can type an exact number directly"""
        param_name = self._param_names[self._selected]
        # Get current value (works for kp/ki/kd)
        current = getattr(self.pid, self._param_names[self._selected].lower())

        new_val = simpledialog.askfloat(
            title="Manual PID Value",
            prompt=f"Enter new {param_name} value:",
            initialvalue=current,
            minvalue=0.0
        )

        if new_val is not None:   # user clicked OK (not Cancel)
            new_val = max(0.0, new_val)   # never allow negative gains
            if self._selected == 0:
                self.pid.kp = new_val
            elif self._selected == 1:
                self.pid.ki = new_val
            else:
                self.pid.kd = new_val
            self._update_label()

    def _print_final_summary(self):
        """Print a beautiful, log-style summary of the final PID gains when closing"""
        print("\n" + "═" * 70)
        print(" " * 22 + "🎯 PID TUNING SESSION COMPLETE 🎯")
        print("═" * 70)
        print(f"{'Final Tuned Gains':^70}")
        print("─" * 70)
        print(f"   Kp  =  {self.pid.kp:8.5f}     ← Proportional")
        print(f"   Ki  =  {self.pid.ki:8.5f}     ← Integral")
        print(f"   Kd  =  {self.pid.kd:8.5f}     ← Derivative")
        print("─" * 70)
        print("✅ Copy these values into your robot code for perfect wall following!")
        print("   (Recommended: hard-code them or save to a config file.)\n")

    def _close_gui(self):
        """Close the GUI and print the fancy final PID summary"""
        if self._root:
            self._print_final_summary()
            try:
                self._root.destroy()
            except:
                pass
            self._root = None