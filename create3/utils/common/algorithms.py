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
    """Classic RANSAC (Random Sample Consensus) for robust model fitting.

    Repeatedly samples minimal sets of points, fits a model, counts hard inliers
    (points below `config.distance_threshold`), and keeps the model with the
    most inliers. After finding the best model it refits on all inliers and
    segments the result into shape-specific primitives (lines or circles).

    This is the standard, simple, and fast RANSAC variant.

    Parameters
    ----------
    points : list[tuple[float, float]]
        Input point cloud.
    config : RansacConfig
        Configuration (max_iterations, distance_threshold, min_inliers, etc.).
    min_sample_size : int
        Minimum number of points needed to fit a model (2 for line, 3 for circle).
    fit_model : callable
        Function that takes a minimal sample and returns a model.
    distance_func : callable
        Function that returns distance from a point to the model.
    segment_func : callable
        Function that groups inliers into contiguous segments/arcs.
    build_result_func : callable
        Function that turns a segment + model into a final ResultT object.

    Returns
    -------
    list[ResultT]
        All detected geometric primitives (lines or circles) after segmentation.
    """
    remaining_points = points.copy()
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
                # Hard inlier count
                inliers = [
                    pt for pt in remaining_points
                    if distance_func(pt, *model) < config.distance_threshold
                ]
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_model = model
            except ValueError:
                continue  # bad sample (e.g. vertical line)

        if len(best_inliers) < config.min_inliers or best_model is None:
            break

        # Refit on all inliers for better accuracy
        model = fit_model(best_inliers)

        # Shape-specific segmentation (line segments or circle arcs)
        segments = segment_func(best_inliers, *model, config)

        for seg in segments:
            if len(seg) >= config.min_points:
                result = build_result_func(seg, *model, config)
                if result is not None:
                    results.append(result)

        # Remove used points so we don't detect the same feature twice
        remaining_points = [pt for pt in remaining_points if pt not in best_inliers]

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
    """MSAC (M-Estimator SAmple Consensus) — a more robust variant of RANSAC.

    Instead of simply counting inliers, MSAC minimizes a soft cost:
        cost = Σ min(distance², threshold²)   over ALL points

    Lower cost = better model. This makes it less sensitive to the exact
    choice of threshold and more robust to noise/outliers than classic RANSAC.

    Otherwise the algorithm structure is identical to `ransac`.

    Parameters
    ----------
    (same as ransac)

    Returns
    -------
    list[ResultT]
        All detected geometric primitives (lines or circles) after segmentation.
    """
    remaining_points = points.copy()
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

                # Compute soft MSAC cost over ALL remaining points
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

            except ValueError:
                continue  # bad sample

        if len(best_inliers) < config.min_inliers or best_model is None:
            break

        # Refit on all inliers of the best model
        model = fit_model(best_inliers)

        # Shape-specific segmentation
        segments = segment_func(best_inliers, *model, config)

        for seg in segments:
            if len(seg) >= config.min_points:
                result = build_result_func(seg, *model, config)
                if result is not None:
                    results.append(result)

        # Remove used points
        remaining_points = [pt for pt in remaining_points if pt not in best_inliers]

    return results