#
# RANSAC & MSAC Algorithms for iRobot Create3 - Jazzy
# =====================================================================
# Created by scottcandy34 • Optimized (April 2026)
#
# High-performance robust model fitting for 2D geometric primitives
# (lines and circles) in LiDAR point clouds.
#
# This module is the core of the perception pipeline used by
# WALL_DETECTION and COLUMN_DETECTION tasks.
#
# Key Features:
#   • Classic RANSAC (hard inlier count)
#   • MSAC (soft M-Estimator cost – more robust to noise)
#   • Efficient inlier removal using sets (major speed-up)
#   • Early termination when a model explains all points
#   • Clean, well-documented, production-ready code
#
# Performance Note:
#   Original version was O(n²) in worst case due to repeated
#   "not in list" checks. This version is dramatically faster
#   while remaining pure-Python and easy to understand.
# =====================================================================

import random
from typing import Any, Callable, TypeVar

from create3.models.common import RansacConfig

ResultT = TypeVar("ResultT")

def ransac(
    points: list[tuple[float, float]],
    config: RansacConfig,
    min_sample_size: int,
    fit_model: Callable[[list[tuple[float, float]]], Any],
    distance_func: Callable[..., float],
    segment_func: Callable[..., list[list[tuple[float, float]]]],
    build_result_func: Callable[..., ResultT | None],
) -> list[ResultT]:
    """Classic RANSAC for robust line or circle fitting.

    Repeatedly samples minimal sets of points, fits a model, and counts
    hard inliers (points within `config.distance_threshold`). Keeps the
    best model, refits on all its inliers, then segments them into
    contiguous primitives (lines or arcs).

    This implementation has been heavily optimized for speed:
      • Uses a set for O(1) inlier removal instead of O(n) list lookup
      • Early termination when a model explains all remaining points
      • Minimal list copying

    Parameters
    ----------
    points : list[tuple[float, float]]
        Input point cloud (x, y) in world coordinates.
    config : RansacConfig
        RANSAC parameters (max_iterations, distance_threshold, min_inliers, etc.).
    min_sample_size : int
        Minimum points needed to fit a model (2 for line, 3 for circle).
    fit_model : callable
        Function that fits a model to a minimal sample.
    distance_func : callable
        Returns distance from a point to the model.
    segment_func : callable
        Groups inliers into contiguous segments/arcs.
    build_result_func : callable
        Converts a segment + model into a final result object.

    Returns
    -------
    list[ResultT]
        All detected geometric primitives (Wall or Column objects).
    """
    if len(points) < config.min_inliers:
        return []

    remaining_points = points[:]  # shallow copy is sufficient
    results: list[ResultT] = []

    while len(remaining_points) >= config.min_inliers:
        best_inliers: list[tuple[float, float]] = []
        best_model = None

        for _ in range(config.max_iterations):
            if len(remaining_points) < min_sample_size:
                break

            sample = random.sample(remaining_points, min_sample_size)

            try:
                model = fit_model(sample)
                # Hard inlier test
                inliers = [pt for pt in remaining_points if distance_func(pt, *model) < config.distance_threshold]

                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_model = model

                    # Early exit: perfect model found
                    if len(best_inliers) == len(remaining_points):
                        break

            except ValueError:
                continue  # degenerate sample (e.g. vertical line)

        if len(best_inliers) < config.min_inliers or best_model is None:
            break

        # Refit on all inliers for higher accuracy
        model = fit_model(best_inliers)

        # Segment inliers into contiguous primitives
        segments = segment_func(best_inliers, *model, config)

        for seg in segments:
            if len(seg) >= config.min_points:
                result = build_result_func(seg, *model, config)
                if result is not None:
                    results.append(result)

        # Efficient removal of used inliers
        inlier_set = set(best_inliers)  # tuples are hashable → O(1) lookup
        remaining_points = [pt for pt in remaining_points if pt not in inlier_set]

    return results

def msac(
    points: list[tuple[float, float]],
    config: RansacConfig,
    min_sample_size: int,
    fit_model: Callable[[list[tuple[float, float]]], Any],
    distance_func: Callable[..., float],
    segment_func: Callable[..., list[list[tuple[float, float]]]],
    build_result_func: Callable[..., ResultT | None],
) -> list[ResultT]:
    """MSAC (M-Estimator SAmple Consensus) – a more robust variant of RANSAC.

    Instead of counting inliers, MSAC minimizes a soft cost:
        cost = Σ min(distance², threshold²) over ALL points

    This makes it significantly less sensitive to the exact threshold value
    and more robust to outliers than classic RANSAC.

    Same performance optimizations as `ransac()` are applied.
    """
    if len(points) < config.min_inliers:
        return []

    remaining_points = points[:]
    results: list[ResultT] = []

    T2 = config.distance_threshold**2  # squared threshold for MSAC cost

    while len(remaining_points) >= config.min_inliers:
        best_cost = float("inf")
        best_inliers: list[tuple[float, float]] = []
        best_model = None

        for _ in range(config.max_iterations):
            if len(remaining_points) < min_sample_size:
                break

            sample = random.sample(remaining_points, min_sample_size)

            try:
                model = fit_model(sample)

                total_cost = 0.0
                inliers_for_this_model: list[tuple[float, float]] = []

                for pt in remaining_points:
                    dist = distance_func(pt, *model)
                    total_cost += min(dist * dist, T2)
                    if dist < config.distance_threshold:
                        inliers_for_this_model.append(pt)

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_inliers = inliers_for_this_model
                    best_model = model

                    # Early exit on perfect model
                    if len(best_inliers) == len(remaining_points):
                        break

            except ValueError:
                continue

        if len(best_inliers) < config.min_inliers or best_model is None:
            break

        # Refit and segment
        model = fit_model(best_inliers)
        segments = segment_func(best_inliers, *model, config)

        for seg in segments:
            if len(seg) >= config.min_points:
                result = build_result_func(seg, *model, config)
                if result is not None:
                    results.append(result)

        # Efficient removal
        inlier_set = set(best_inliers)
        remaining_points = [pt for pt in remaining_points if pt not in inlier_set]

    return results