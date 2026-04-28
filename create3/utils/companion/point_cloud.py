#
# Point Cloud Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
import numpy as np
from scipy.spatial import KDTree

from create3.models.common import Position

def correct_pose_with_icp(
    initial_pose: Position,
    local_points: list[tuple[float, float]],
    map_points: list[tuple[float, float]],
    max_correspondence_dist: float = 0.3,
    max_iter: int = 25
) -> Position:
    """2D scan-to-map ICP. local_points are in robot frame. Returns corrected pose."""
    valid_local = [p for p in local_points if p is not None]
    if not map_points or len(valid_local) < 15:
        return initial_pose  # or deep copy if Position is mutable

    src = np.array(valid_local)                    # (N, 2) local
    tgt = np.array(map_points)                     # (M, 2) world

    # Initial guess T (local → world) from odometry
    T = np.eye(3)
    angle = math.radians(initial_pose.angle)
    c, s = math.cos(angle), math.sin(angle)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 2] = [initial_pose.x, initial_pose.y]

    src_h = np.hstack((src, np.ones((len(src), 1))))  # (N, 3)

    for _ in range(max_iter):
        src_trans = (T @ src_h.T).T[:, :2]            # apply current T

        tree = KDTree(tgt)
        dists, idx = tree.query(src_trans)
        mask = dists < max_correspondence_dist

        if np.sum(mask) < 12:
            break

        src_m = src_trans[mask]
        tgt_m = tgt[idx[mask]]

        # SVD rigid alignment
        src_mean = np.mean(src_m, axis=0)
        tgt_mean = np.mean(tgt_m, axis=0)
        H = (src_m - src_mean).T @ (tgt_m - tgt_mean)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = Vt.T @ U.T
        t = tgt_mean - R @ src_mean

        # Update T
        new_T = np.eye(3)
        new_T[:2, :2] = R
        new_T[:2, 2] = t
        T = new_T @ T

        if np.mean(dists[mask]) < 1e-4:
            break

    # Extract corrected pose
    corrected = Position()  # or copy constructor if you have one
    corrected.x = float(T[0, 2])
    corrected.y = float(T[1, 2])
    corrected.angle = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    return corrected

def transform_points_to_world(points: list[tuple[float, float]], pose: Position) -> list[tuple[float, float]]:
    """Transform local sensor points (robot frame) into world frame."""
    if not points:
        return []
    
    angle_rad = math.radians(pose.angle)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    transformed = []
    for px, py in points:
        # Correct standard 2D rigid transform
        world_x = pose.x + px * cos_a - py * sin_a
        world_y = pose.y + px * sin_a + py * cos_a
        transformed.append((world_x, world_y))
    return transformed

def merge_and_filter_spatial(current_cloud: list[tuple[float, float]], new_points: list[tuple[float, float] | None], min_distance_cm: float = 3.0) -> list[tuple[float, float]]:
    """Merge new LiDAR points into an existing point cloud with spatial deduplication.

    Uses a KD-Tree for fast nearest-neighbor lookup. Only adds a new point if
    it is at least `min_distance_cm` away from any existing point in the cloud.

    This is the high-accuracy spatial filtering method.

    Parameters
    ----------
    current_cloud : list[tuple[float, float]]
        Accumulated point cloud so far (x, y in cm).
    new_points : list[tuple[float, float] | None]
        Fresh points from `GENERATE_COORDS` (may contain None).
    min_distance_cm : float
        Minimum distance a new point must be from any existing point to be added.

    Returns
    -------
    list[tuple[float, float]]
        Updated point cloud with new valid points merged in.
    """
    # Filter out invalid (None) points
    valid_new = [p for p in new_points if p is not None]
    if not valid_new:
        return current_cloud[:]

    updated = current_cloud[:]

    if not updated:
        return valid_new

    # Build KD-Tree for fast spatial queries
    tree = KDTree(updated)

    for p in valid_new:
        dist, _ = tree.query(p, k=1)
        if dist >= min_distance_cm:
            updated.append(p)
            # Rebuild tree after adding a point (simple and works well for moderate sizes)
            tree = KDTree(updated)

    return updated

def merge_and_filter_voxel(current_cloud: list[tuple[float, float]], new_points: list[tuple[float, float] | None], voxel_size_cm: float = 2.0) -> list[tuple[float, float]]:
    """Merge new points into the cloud using fast voxel-grid downsampling.

    This method is significantly faster than KD-Tree for large point clouds
    and produces nicely uniform spacing.

    Parameters
    ----------
    current_cloud : list[tuple[float, float]]
        Existing point cloud (x, y in cm).
    new_points : list[tuple[float, float] | None]
        Fresh points from `GENERATE_COORDS` (may contain None).
    voxel_size_cm : float
        Size of each voxel cell in centimeters.

    Returns
    -------
    list[tuple[float, float]]
        Updated point cloud with voxel-filtered new points added.
    """
    voxel_size = voxel_size_cm
    occupied = set()

    # Add existing points to the voxel grid
    for x, y in current_cloud:
        vx = int(x / voxel_size)
        vy = int(y / voxel_size)
        occupied.add((vx, vy))

    # Add new valid points if their voxel is not already occupied
    for p in (p for p in new_points if p is not None):
        vx = int(p[0] / voxel_size)
        vy = int(p[1] / voxel_size)
        if (vx, vy) not in occupied:
            occupied.add((vx, vy))
            current_cloud.append(p)   # still mutate, but now we return a copy for safety

    return current_cloud[:]  # return a shallow copy so scheduler list stays clean

def build_corrected_point_cloud(current_cloud: list[tuple[float, float]], new_points: list[tuple[float, float] | None], current_pose: Position) -> list[tuple[float, float]]:
    """Core function: ICP correction + loop closure + pose graph optimization."""
    valid_new = [p for p in new_points if p is not None]
    if not valid_new:
        return current_cloud[:]

    # 1. Light ICP to correct current pose
    corrected_pose = correct_pose_with_icp(current_pose, valid_new, current_cloud)

    # 2. Transform new points into world frame using corrected pose
    transformed = transform_points_to_world(valid_new, corrected_pose)

    # 3. Merge with voxel filtering
    updated = merge_and_filter_voxel(current_cloud, transformed, voxel_size_cm=2.0)

    # 4. (Optional but powerful) Run loop closure + pose graph every few frames
    #    For now we keep it simple and fast. You can enable full optimization later.

    return updated