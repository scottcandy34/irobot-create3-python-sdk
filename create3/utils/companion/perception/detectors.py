#
# Detector Tools for iRobot Create3 - Jazzy
# =====================================================================
# Created by scottcandy34 • Fully Revised & Documented April 2026
#
# High-level geometric feature detectors for LiDAR point clouds.
#
# This is the main entry point for perception in the companion node.
# It uses the highly optimized MSAC + NumPy backend to robustly detect:
#   • Straight wall segments (WALL_DETECTION)
#   • Circular columns / obstacles (COLUMN_DETECTION)
#
# Recommended usage:
#   detections = find_walls_and_columns(deskewed_points)
# =====================================================================

from create3.utils.common import algorithms
from create3.models.common import RansacConfig
from create3.models.companion import Wall, Column, Detections

# Updated perception modules (NumPy-optimized)
from . import line_segments, lines, circle_arcs, circles

# =====================================================================
# Global RANSAC/MSAC Configurations
# =====================================================================

line_ransac_config = RansacConfig(
    max_iterations=100,
    distance_threshold=1.0,   # cm
    min_inliers=30,
    max_gap=5.0,              # cm along line
    min_points=30,
)

circle_ransac_config = RansacConfig(
    max_iterations=100,
    distance_threshold=1.0,   # cm
    min_inliers=30,
    max_gap=15.0,             # degrees angular gap
    min_points=30,
)


def find_line_segments(points: list[tuple[float, float]]) -> list[Wall]:
    """Detect straight wall segments in a 2D point cloud using MSAC.

    This is the recommended high-level entry point for line detection.
    Uses the optimized MSAC algorithm (more robust than classic RANSAC).

    Parameters
    ----------
    points : list[tuple[float, float]]
        Raw world-frame point cloud (x, y) from GENERATE_COORDS task.

    Returns
    -------
    list[Wall]
        List of detected wall segments. Empty list if none found.
    """
    return _find_lines(points, line_ransac_config)


def find_circle_arcs(points: list[tuple[float, float]]) -> list[Column]:
    """Detect circular arcs (columns/obstacles) in a 2D point cloud using MSAC.

    This is the recommended high-level entry point for circle detection.

    Parameters
    ----------
    points : list[tuple[float, float]]
        Raw world-frame point cloud (x, y) from GENERATE_COORDS task.

    Returns
    -------
    list[Column]
        List of detected circular arcs/columns. Empty list if none found.
    """
    return _find_circles(points, circle_ransac_config)


def find_walls_and_columns(points: list[tuple[float, float]]) -> Detections:
    """Detect both walls and columns from a single point cloud in one call.

    This is the main convenience function used by most tasks and visualizers.

    Parameters
    ----------
    points : list[tuple[float, float]]
        Raw world-frame point cloud (typically from deskewed LiDAR).

    Returns
    -------
    Detections
        Container with `.walls` and `.columns` lists.
    """
    walls = find_line_segments(points)
    columns = find_circle_arcs(points)
    return Detections(columns=columns, walls=walls)


# =====================================================================
# Internal optimized implementations
# =====================================================================

def _find_lines(
    points: list[tuple[float, float]], config: RansacConfig
) -> list[Wall]:
    """Internal MSAC-based line segment detector (used by public API)."""

    def build_wall(
        segment: list[tuple[float, float]], m: float, b: float, cfg: RansacConfig
    ) -> Wall:
        """Convert a line segment + model parameters into a Wall object."""
        proj_first = lines.project_point(segment[0], m, b)
        proj_last = lines.project_point(segment[-1], m, b)

        xmin = min(proj_first[0], proj_last[0])
        xmax = max(proj_first[0], proj_last[0])
        length = lines.calculate_length(segment, m, b)

        return Wall(
            length=length,
            slope=m,
            intercept=b,
            xmin=xmin,
            xmax=xmax,
        )

    return algorithms.msac(
        points=points,
        config=config,
        min_sample_size=2,
        fit_model=lines.fit_line,
        distance_func=lines.distance_to_line,
        segment_func=lambda pts, m, b, cfg: line_segments.find(
            pts, m, b, cfg.max_gap, min_points=cfg.min_points
        ),
        build_result_func=build_wall,
    )


def _find_circles(
    points: list[tuple[float, float]], config: RansacConfig
) -> list[Column]:
    """Internal MSAC-based circle/arc detector (used by public API)."""

    def build_column(
        arc_points: list[tuple[float, float]],
        cx: float,
        cy: float,
        r: float,
        cfg: RansacConfig,
    ) -> Column:
        """Convert a circle arc + model parameters into a Column object."""
        start_angle, end_angle = circles.get_angle_range(arc_points, cx, cy)
        arc_length = circles.calculate_arc_length(arc_points, cx, cy, r)

        return Column(
            cx=cx,
            cy=cy,
            radius=r,
            start_angle=start_angle,
            end_angle=end_angle,
            arc_length=arc_length,
        )

    return algorithms.msac(
        points=points,
        config=config,
        min_sample_size=3,
        fit_model=circles.fit_circle,
        distance_func=circles.distance_to_circle,
        segment_func=lambda pts, cx, cy, r, cfg: circle_arcs.find(
            pts, cx, cy, r, cfg.max_gap, min_points=cfg.min_points
        ),
        build_result_func=build_column,
    )