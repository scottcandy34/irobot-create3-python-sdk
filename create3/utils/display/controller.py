#
# iRobot Create3 - Controller Visualizer (Live PS Controller Overlay)
# =====================================================================
# Created by scottcandy34 – heavily extended and commented for you
#
# WHAT THIS DOES
# ==============
# Real-time visual overlay of your PlayStation-style controller while
# using the remote node. Perfect for debugging controller input or
# demonstrating the remote control system.
#
# WHAT YOU WILL SEE
# =================
# • Left & Right analog joysticks (green dots move in real time)
# • Left & Right triggers (L2 / R2) as growing bars
# • All face buttons (X, Circle, Square, Triangle) light up bright green when pressed
# • Shoulder buttons (L1, R1), Share, Options, and PS button
# • Full D-pad with individual directional highlights
#
# HOW TO RUN
# ==========
# 1. Make sure your RemoteNode is running (it must be publishing controller data)
# 2. Run this file:
#
#    python examples/controller_visualizer.py
#
# 3. The window will appear and update every 50 ms
# 4. Press any button or move the sticks — you’ll see it instantly
#
# TIPS
# ====
# • The visualizer runs independently of the robot — you can use it even
#   while the robot is driving.
# • Close the window with the X or press Esc (if you added that later).
# • Great for presentations or verifying that your controller mapping is correct.
#
# =====================================================================

from pathlib import Path
import tkinter as tk

from create3 import RemoteNode

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("⚠️  Pillow not installed. Install with: pip install pillow")

class ControllerVisualizer:
    """Real-time visualizer for a PlayStation-style controller.

    Displays a live overlay of the controller state (joysticks, triggers,
    buttons, D-pad) on top of a background image of the controller.

    The visualizer updates every 50 ms using the latest data from
    `remote.get_controller()`.
    """

    def __init__(self, remote: "RemoteNode") -> None:
        """Launch the controller visualizer window.

        Parameters
        ----------
        remote : RemoteNode
            The remote node providing controller input via `get_controller()`.
        """
        self.remote = remote

        self.root = tk.Tk()
        self.root.title("iRobot Create3 - Controller Visualizer")
        self.root.configure(bg="#111111")
        self.root.resizable(False, False)

        # Load and scale background image, then create overlays
        self._load_background(max_height=600)
        self._create_overlays()

        # Start real-time update loop
        self._update()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _load_background(self, max_height: int = 600) -> None:
        """Load the controller background image and scale it to fit the window.

        Falls back to a plain canvas if PIL is not installed or the image is missing.
        """
        image_path = Path(__file__).parent / "ps4.jpg"

        if HAS_PIL and image_path.exists():
            try:
                pil_image = Image.open(image_path)
                original_w, original_h = pil_image.size

                # Scale so height <= max_height while preserving aspect ratio
                scale = max_height / original_h
                new_w = int(original_w * scale)
                new_h = int(original_h * scale)

                pil_image = pil_image.resize((new_w, new_h), Image.Resampling.LANCZOS)
                self.controller_image = ImageTk.PhotoImage(pil_image)

                self.canvas = tk.Canvas(
                    self.root,
                    width=new_w,
                    height=new_h,
                    bg="#111111",
                    highlightthickness=0,
                )
                self.canvas.pack()
                self.canvas.create_image(0, 0, anchor=tk.NW, image=self.controller_image)

                self.scale_factor = scale
                return

            except Exception as e:
                print(f"⚠️ Could not load controller image: {e}")

        # Fallback: plain dark canvas
        self.canvas = tk.Canvas(self.root, width=900, height=520, bg="#222222", highlightthickness=0)
        self.canvas.pack()
        self.scale_factor = 1.0

    def _create_overlays(self) -> None:
        """Create all visual overlays (joysticks, triggers, buttons, D-pad)."""
        s = self.scale_factor

        # Left joystick dot
        self.left_dot = self.canvas.create_oval(
            399 * s - 20, 540 * s - 20, 399 * s + 20, 540 * s + 20,
            fill="#00ff88", outline="#00cc66", width=3
        )

        # Right joystick dot
        self.right_dot = self.canvas.create_oval(
            723 * s - 20, 538 * s - 20, 723 * s + 20, 538 * s + 20,
            fill="#00ff88", outline="#00cc66", width=3
        )

        # Left trigger bar (L2)
        self.left_trigger_bar = self.canvas.create_rectangle(
            313 * s, 83 * s, 313 * s + 55 * s, 113 * s,
            fill="#00ff88", outline="#00cc66"
        )

        # Right trigger bar (R2)
        self.right_trigger_bar = self.canvas.create_rectangle(
            807 * s, 84 * s, 807 * s + 55 * s, 114 * s,
            fill="#00ff88", outline="#00cc66"
        )

        # Button overlays (filled only when pressed)
        self.button_overlays = {}

        # Face buttons (X, Circle, Square, Triangle)
        action_config = {
            "triangle": (874 * s, 331 * s, 28 * s),
            "circle":   (947 * s, 404 * s, 28 * s),
            "x":        (874 * s, 476 * s, 28 * s),
            "square":   (803 * s, 403 * s, 28 * s),
        }
        for name, (cx, cy, r) in action_config.items():
            item = self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r,
                fill="", outline="", width=3
            )
            self.button_overlays[name] = item

        # PS button
        ps_cx, ps_cy = 561 * s, 547 * s
        ps_r = 19 * s
        self.ps_overlay = self.canvas.create_oval(
            ps_cx - ps_r, ps_cy - ps_r, ps_cx + ps_r, ps_cy + ps_r,
            fill="", outline="", width=3
        )
        self.button_overlays["ps"] = self.ps_overlay

        # Share button
        share_cx, share_cy = 354 * s, 306 * s
        self.share_overlay = self.canvas.create_rectangle(
            share_cx - 28 * s, share_cy - 13 * s,
            share_cx + 18 * s, share_cy + 13 * s,
            fill="", outline="", width=3
        )
        self.button_overlays["share"] = self.share_overlay

        # Options button
        opt_cx, opt_cy = 767 * s, 303 * s
        self.options_overlay = self.canvas.create_rectangle(
            opt_cx - 18 * s, opt_cy - 13 * s,
            opt_cx + 28 * s, opt_cy + 13 * s,
            fill="", outline="", width=3
        )
        self.button_overlays["options"] = self.options_overlay

        # L1 button
        l1_cx, l1_cy = 295 * s, 168 * s
        self.l1_overlay = self.canvas.create_rectangle(
            l1_cx - 45 * s, l1_cy - 18 * s,
            l1_cx + 35 * s, l1_cy + 12 * s,
            fill="", outline="", width=3
        )
        self.button_overlays["l1"] = self.l1_overlay

        # R1 button
        r1_cx, r1_cy = 827 * s, 168 * s
        self.r1_overlay = self.canvas.create_rectangle(
            r1_cx - 35 * s, r1_cy - 18 * s,
            r1_cx + 45 * s, r1_cy + 12 * s,
            fill="", outline="", width=3
        )
        self.button_overlays["r1"] = self.r1_overlay

        # D-pad (individual directional overlays)
        dpad_r = 23 * s
        self.dpad_overlays = {
            "up": self.canvas.create_oval(
                246 * s - dpad_r, 350 * s - dpad_r,
                246 * s + dpad_r, 350 * s + dpad_r,
                fill="", outline="", width=2
            ),
            "down": self.canvas.create_oval(
                244 * s - dpad_r, 454 * s - dpad_r,
                244 * s + dpad_r, 454 * s + dpad_r,
                fill="", outline="", width=2
            ),
            "left": self.canvas.create_oval(
                194 * s - dpad_r, 400 * s - dpad_r,
                194 * s + dpad_r, 400 * s + dpad_r,
                fill="", outline="", width=2
            ),
            "right": self.canvas.create_oval(
                295 * s - dpad_r, 404 * s - dpad_r,
                295 * s + dpad_r, 404 * s + dpad_r,
                fill="", outline="", width=2
            ),
        }

    def _update(self) -> None:
        """Update all visual overlays with the latest controller state (50 ms interval)."""
        ctrl = self.remote.get_controller()
        s = self.scale_factor

        # Joysticks
        lx = 399 * s - ctrl.left_joy.horizontal * 55 * s
        ly = 540 * s - ctrl.left_joy.vertical * 55 * s
        self.canvas.coords(self.left_dot, lx - 20, ly - 20, lx + 20, ly + 20)

        rx = 723 * s - ctrl.right_joy.horizontal * 55 * s
        ry = 538 * s - ctrl.right_joy.vertical * 55 * s
        self.canvas.coords(self.right_dot, rx - 20, ry - 20, rx + 20, ry + 20)

        # Analog triggers (bars)
        self.canvas.coords(
            self.left_trigger_bar,
            313 * s,
            83 * s,
            313 * s + ctrl.left_trigger * 55 * s,
            113 * s,
        )
        self.canvas.coords(
            self.right_trigger_bar,
            807 * s,
            84 * s,
            807 * s + ctrl.right_trigger * 55 * s,
            114 * s,
        )

        # Face / special buttons
        for name, item in self.button_overlays.items():
            pressed = getattr(ctrl.buttons, name, False)
            self.canvas.itemconfig(
                item,
                fill="#00ff88" if pressed else "",
                outline="#00cc66" if pressed else "",
            )

        # D-pad
        for direction, item in self.dpad_overlays.items():
            pressed = getattr(ctrl.dpad, direction, False)
            self.canvas.itemconfig(
                item,
                fill="#00ff88" if pressed else "",
                outline="#00cc66" if pressed else "",
            )

        self.root.after(50, self._update)

    def _on_close(self) -> None:
        """Cleanly close the visualizer window."""
        self.root.destroy()