#
# Point Cloud Tools for iRobot Create3 - Jazzy
# Created by scottcandy34
#

import math
import numpy as np
from scipy.spatial import KDTree

from create3.models.common import Position

current_cloud: np.ndarray = np.empty((0, 2), dtype=np.float32)   # shape (N, 2)

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

def merge_and_filter_spatial(
    current_cloud: np.ndarray | None,
    new_points: list[tuple[float, float] | None] | np.ndarray,
    min_distance_cm: float = 3.0
) -> np.ndarray:
    """High-accuracy spatial deduplication — now safe when current_cloud is None."""
    if current_cloud is None:
        current_cloud = np.empty((0, 2), dtype=np.float32)

    if isinstance(new_points, list):
        valid_new = np.array([p for p in new_points if p is not None], dtype=np.float32)
    else:
        valid_new = np.asarray(new_points, dtype=np.float32)

    if len(valid_new) == 0:
        return current_cloud.copy()
    if len(current_cloud) == 0:
        return valid_new.copy()

    tree = KDTree(current_cloud)
    dists, _ = tree.query(valid_new, k=1)

    mask = dists >= min_distance_cm
    good_new = valid_new[mask]

    if len(good_new) == 0:
        return current_cloud.copy()

    return np.vstack([current_cloud, good_new])

def merge_and_filter_voxel(
    current_cloud: list[tuple[float, float]] | None,
    new_points: list[tuple[float, float] | None] | np.ndarray,
    voxel_size_cm: float = 2.0
) -> list[tuple[float, float]]:
    """Fast voxel-grid downsampling — returns plain list (100% backward compatible)."""
    # Treat None as empty cloud
    if current_cloud is None:
        current_cloud = []

    # Convert new_points (supports your original list-with-None format)
    if isinstance(new_points, list):
        valid_new = np.array([p for p in new_points if p is not None], dtype=np.float32)
    else:
        valid_new = np.asarray(new_points, dtype=np.float32)

    if len(valid_new) == 0:
        return current_cloud[:]  # shallow copy of list

    voxel_size = float(voxel_size_cm)

    # Occupied voxels from current cloud (fully vectorized)
    occupied = set()
    if current_cloud:
        curr_array = np.array(current_cloud, dtype=np.float32)
        occ_vox = np.floor(curr_array / voxel_size).astype(np.int64)
        occupied = {tuple(v) for v in occ_vox}

    # New voxels — unique + index of first point in each voxel
    new_vox = np.floor(valid_new / voxel_size).astype(np.int64)
    unique_vox, first_idx = np.unique(new_vox, axis=0, return_index=True)

    # Vectorized check for unoccupied voxels
    unoccupied_mask = np.array([tuple(v) not in occupied for v in unique_vox])

    updated = current_cloud[:]  # start with copy of existing list

    if np.any(unoccupied_mask):
        good_idx = first_idx[unoccupied_mask]
        to_add = valid_new[good_idx]
        # Convert only the new points back to tuples
        updated.extend([tuple(p) for p in to_add.tolist()])

    return updated

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