#
# Detector Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

from create3.utils.common import algorithms
from create3.models.common import RansacConfig
from create3.models.companion import Wall, Column, Detections
from . import line_segments, lines, circle_arcs, circles

line_ransac_config = RansacConfig(max_iterations=100, distance_threshold=1.0, min_inliers=30, max_gap=5.0, min_points=30)
circle_ransac_config = RansacConfig( max_iterations=100, distance_threshold=1.0, min_inliers=30, max_gap=15.0, min_points=30)

def find_line_segments(points: list[tuple[float, float]]) -> list[Wall]:
    """Detect straight wall segments in a 2D point cloud using RANSAC + contiguous grouping.

    This is the high-level entry point for line detection. It uses the global
    `line_ransac_config` and returns fully-formed `Wall` objects ready for
    navigation, collision checking, or mapping.

    Internally it:
      • Runs classic RANSAC (or MSAC if you change the call)
      • Fits a line model
      • Groups inliers into contiguous segments along the line
      • Builds a `Wall` with length, endpoints, and line equation

    Parameters
    ----------
    points : list[tuple[float, float]]
        Raw (x, y) point cloud (usually from LiDAR or vision).

    Returns
    -------
    list[Wall]
        List of detected wall segments. Empty list if no valid walls found.
    """
    def build_wall(segment: list[tuple[float, float]], m: float, b: float, cfg: RansacConfig) -> Wall:
        """Convert a line segment + model into a Wall object."""
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

    return algorithms.ransac(   # ← change to algorithms.msac here if you prefer MSAC
        points=points,
        config=line_ransac_config,
        min_sample_size=2,
        fit_model=lines.fit_line,
        distance_func=lines.distance_to_line,
        segment_func=lambda pts, m, b, cfg: line_segments.find(pts, m, b, cfg.max_gap, min_points=cfg.min_points),
        build_result_func=build_wall,
    )

def find_circle_arcs(points: list[tuple[float, float]]) -> list[Column]:
    """Detect circular arcs (columns/obstacles) in a 2D point cloud using RANSAC + angular grouping.

    This is the high-level entry point for circle/arc detection. It uses the global
    `circle_ransac_config` and returns fully-formed `Column` objects ready for
    navigation or mapping.

    Internally it:
      • Runs classic RANSAC (or MSAC if you change the call)
      • Fits a circle model
      • Groups inliers into contiguous arcs
      • Builds a `Column` with center, radius, angular span, and arc length

    Parameters
    ----------
    points : list[tuple[float, float]]
        Raw (x, y) point cloud (usually from LiDAR or vision).

    Returns
    -------
    list[Column]
        List of detected circular arcs/columns. Empty list if none found.
    """
    def build_column(arc_points: list[tuple[float, float]], cx: float, cy: float, r: float, cfg: RansacConfig) -> Column:
        """Convert a circle arc + model into a Column object."""
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

    return algorithms.ransac(   # ← change to algorithms.msac here if you prefer MSAC
        points=points,
        config=circle_ransac_config,
        min_sample_size=3,
        fit_model=circles.fit_circle,
        distance_func=circles.distance_to_circle,
        segment_func=lambda pts, cx, cy, r, cfg: circle_arcs.find(pts, cx, cy, r, cfg.max_gap, min_points=cfg.min_points),
        build_result_func=build_column,
    )

def find_walls_and_columns(points: list[tuple[float, float]]) -> Detections:
    """Detect both straight wall segments and circular columns/arcs from a single 2D point cloud in one call.

    This is the main high-level entry point for geometric feature detection.
    It runs the complete RANSAC pipeline for lines (`find_line_segments`)
    and circles (`find_circle_arcs`) and packages the results into a single
    `Detections` container for easy use in navigation, mapping, or obstacle
    avoidance.

    Parameters
    ----------
    points : list[tuple[float, float]]
        Raw (x, y) point cloud (typically from LiDAR transformed to world frame).

    Returns
    -------
    Detections
        Container with:
        - .walls   : list[_Wall]     — detected straight wall segments
        - .columns : list[_Column]   — detected circular arcs/columns
    """
    walls = find_line_segments(points)
    columns = find_circle_arcs(points)

    return Detections(columns=columns, walls=walls)