#
# Wall Detections Visualizer for iRobot Create3 - Jazzy
# =====================================================================
# Created by scottcandy34 • Revised April 2026
#
# Real-time visualization of detected wall segments.
#
# Key Features:
#   • Robot's front (+X axis) always points UP at the top of the screen
#   • Clean left sidebar listing with colored lines + exact wall lengths
#   • No overlapping text on the map — much easier to read
#   • High-performance rendering
# =====================================================================

import math
import tkinter as tk
from typing import Callable, List, Optional

from create3 import RobotNode
from create3.models.companion import Wall


class DetectionsVisualizer:
    """High-performance real-time visualizer for wall detections.

    Uses a clean left sidebar listing with colored entries instead of
    on-map labels. This eliminates overlap and makes matching easy.
    """

    MAX_WALLS = 12
    UPDATE_MS = 80
    SIDEBAR_WIDTH = 180

    def __init__(
        self,
        robot: RobotNode,
        get_walls: Optional[Callable[[], List[Wall] | None]] = None,
    ) -> None:
        """Launch the wall detections visualizer."""
        self.robot = robot
        self.get_walls = get_walls

        self.root = tk.Tk()
        self.root.title("iRobot Create3 - Wall Detections Visualizer")
        self.root.configure(bg="#0a0a0a")
        self.root.resizable(False, False)

        self.canvas = tk.Canvas(
            self.root,
            width=840 + self.SIDEBAR_WIDTH,
            height=840,
            bg="#111111",
            highlightthickness=0
        )
        self.canvas.pack(padx=20, pady=20)

        self.wall_items: list[int] = []
        self.robot_item: int | None = None
        self.legend_items: list[int] = []
        self.list_items: list[int] = []

        self._update()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _update(self) -> None:
        """Redraw map + clean sidebar list."""
        walls = self.get_walls() if self.get_walls else []
        pose = self.robot.get_position()

        cx, cy = 420 + self.SIDEBAR_WIDTH // 2, 420   # center of map area
        scale = 3.0

        # Clear previous items
        for item in self.wall_items + self.list_items + self.legend_items:
            self.canvas.delete(item)
        if self.robot_item:
            self.canvas.delete(self.robot_item)

        self.wall_items = []
        self.list_items = []
        colors = ["#ff5555", "#55ff55", "#ffaa33", "#aa66ff"]

        # === 1. Draw walls on map ===
        for i, wall in enumerate(walls[:self.MAX_WALLS]):
            color = colors[i % len(colors)]

            x1, y1 = wall.xmin, wall.slope * wall.xmin + wall.intercept
            x2, y2 = wall.xmax, wall.slope * wall.xmax + wall.intercept

            dx1 = x1 - pose.x
            dy1 = y1 - pose.y
            dx2 = x2 - pose.x
            dy2 = y2 - pose.y

            rx1 = -dy1
            ry1 = dx1
            rx2 = -dy2
            ry2 = dx2

            px1 = cx + rx1 * scale
            py1 = cy - ry1 * scale
            px2 = cx + rx2 * scale
            py2 = cy - ry2 * scale

            line = self.canvas.create_line(px1, py1, px2, py2,
                                           fill=color, width=5)
            self.wall_items.append(line)

        # === 2. Draw robot (centered in map area) ===
        self.robot_item = self._draw_robot(cx, cy)

        # === 3. Draw sidebar list ===
        self._draw_sidebar_list(walls, colors)

        # === 4. Draw main legend ===
        self._draw_legend()

        self.root.after(self.UPDATE_MS, self._update)

    def _draw_sidebar_list(self, walls: List[Wall], colors: list[str]) -> None:
        """Draw clean vertical list on the left with colored lines + lengths."""
        x = 30
        y = 120

        for i, wall in enumerate(walls[:self.MAX_WALLS]):
            color = colors[i % len(colors)]

            # Colored line
            line = self.canvas.create_line(x, y, x + 60, y, fill=color, width=4)
            self.list_items.append(line)

            # Length text
            text = self.canvas.create_text(
                x + 80, y,
                text=f"{wall.length:.0f} cm",
                fill=color,
                font=("Consolas", 12, "bold"),
                anchor="w"
            )
            self.list_items.append(text)

            y += 42

    def _draw_robot(self, cx: float, cy: float) -> int:
        """Draw robot as green triangle pointing straight up."""
        size = 20
        x1 = cx
        y1 = cy - size
        x2 = cx - size * 0.7
        y2 = cy + size * 0.6
        x3 = cx + size * 0.7
        y3 = cy + size * 0.6

        return self.canvas.create_polygon(
            x1, y1, x2, y2, x3, y3,
            fill="#00ff88", outline="#00cc66", width=3
        )

    def _draw_legend(self) -> None:
        """Main title."""
        for item in self.legend_items:
            self.canvas.delete(item)
        self.legend_items = []

        self.legend_items.append(
            self.canvas.create_text(30, 35,
                                    text="WALL DETECTIONS VISUALIZER",
                                    fill="#ffffff",
                                    font=("Consolas", 14, "bold"),
                                    anchor="w")
        )

    def _on_close(self) -> None:
        self.root.destroy()