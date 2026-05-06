#
# Perception Tools for iRobot Create3 - Jazzy
# =====================================================================
# Created by scottcandy34 • Fully Revised & Documented April 2026
#
# High-level perception package for processing LiDAR point clouds.
#
# This package contains all geometric perception utilities used by the
# companion node, including:
#   • Robust line and circle fitting (RANSAC / MSAC)
#   • Wall segment and column/arc detection
#   • Contiguous segment/arc grouping
#   • Collision prediction between robot and walls
#   • Vectorized NumPy-optimized helper functions
#
# All detectors are highly optimized for real-time performance on
# the companion computer (Raspberry Pi).
#
# Recommended public API:
#   from create3.utils.companion.perception import (
#       find_walls_and_columns,
#       find_line_segments,
#       find_circle_arcs,
#       circle_to_wall_distance
#   )
# =====================================================================

"""
Perception submodule for the iRobot Create3 companion node.

Provides a complete set of tools for converting raw LiDAR scans into
high-level geometric features (walls and columns) and performing
predictive collision checks.

All core functions are NumPy-accelerated and use the optimized MSAC
algorithm for maximum robustness in real-world noisy environments.
"""

# Public API re-exports for convenience
from .detectors import (
    find_line_segments,
    find_circle_arcs,
    find_walls_and_columns,
)
from .collisions import circle_to_wall_distance

# Internal modules (still available for advanced use)
from . import (
    lines,
    line_segments,
    circles,
    circle_arcs,
    collisions,
    detectors,
)

__all__ = [
    "find_line_segments",
    "find_circle_arcs",
    "find_walls_and_columns",
    "circle_to_wall_distance",
    # internal modules
    "lines",
    "line_segments",
    "circles",
    "circle_arcs",
    "collisions",
    "detectors",
]