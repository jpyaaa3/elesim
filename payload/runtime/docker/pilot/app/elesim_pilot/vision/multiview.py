"""Bounded static-object scan geometry; no camera, actuator or workflow I/O.

The map is anchored to the first optical frame. FK provides a relative-pose
initialization, overlap registration is local and bounded, and original sensor
frames remain the caller's property. A saturated scan is not proof of unseen
surface completeness. The caller must keep the object and arm base stationary.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
from scipy.optimize import least_squares

from .floor_calibration import camera_transform
from .partial_cloud import VoxelCloud


def _points(points):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) > 100_000 or not np.all(np.isfinite(p)):
        raise ValueError('invalid scan points')
    return p


def transform(points, pose):
    p = _points(points)
    t = camera_transform(pose)
    return p @ t[:3, :3].T + t[:3, 3]


def voxel_points(points, voxel_m=.003):
    cloud = VoxelCloud(voxel_m=voxel_m, max_voxels=100_000)
    cloud.add(_points(points))
    return cloud.points()


def _surface_normals(points):
    indices = cKDTree(points).query(points, k=min(20, len(points)))[1]
    patch = points[indices]
    centered = patch-patch.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered)/patch.shape[1]
    values, vectors = np.linalg.eigh(covariance)
    usable = values[:, 0] < np.maximum(values[:, 1], 1e-12)*.20
    return vectors[:, :, 0], usable


def register_overlap(source, target, *, max_distance=.025, residual_limit=.0045, min_overlap=.20):
    """Local robust point-to-plane ICP, retaining the FK prior in null space.

    Normals reject correspondences on differently oriented faces. Singular
    directions are not solved from noise on a flat patch. A partial constraint
    may improve a pose without claiming all six directions were observed.
    Registration confidence does not prove object identity or global accuracy.
    """
    source, target = voxel_points(source, .003), voxel_points(target, .003)
    identity = np.eye(4)
    base = {'accepted': False, 'refined': False, 'transform': identity, 'reason': 'insufficient_overlap',
            'overlap': 0., 'residual_p90_m': None, 'iterations': 0}
    if min(len(source), len(target)) < 60:
        return base
    if len(source) > 4000:
        source = source[np.linspace(0, len(source)-1, 4000, dtype=int)]
    source_normal, source_ok = _surface_normals(source)
    normal, target_ok = _surface_normals(target)
    tree = cKDTree(target)
    total = identity.copy()
    center = source.mean(axis=0)
    scale = max(float(np.linalg.norm(np.std(source, axis=0))), .02)
    rank = 0
    for iteration in range(35):
        current = transform(source, total)
        distance, indices = tree.query(current)
        rotated_normal = source_normal @ total[:3, :3].T
        agreement = np.abs(np.sum(rotated_normal*normal[indices], axis=1))
        kept = (distance < max_distance) & source_ok & target_ok[indices] & (agreement > .85)
        if kept.sum() < 60 or kept.mean() < min_overlap:
            return {**base, 'overlap': float(kept.mean()), 'iterations': iteration}
        p, n = current[kept], normal[indices[kept]]
        residual = np.sum((p-target[indices[kept]])*n, axis=1)
        weights = np.minimum(1., .002/np.maximum(np.abs(residual), 1e-9))
        a = np.column_stack([np.cross(p-center, n)/scale, n])
        weighted = a*np.sqrt(weights[:, None])
        u, singular, vt = np.linalg.svd(weighted, full_matrices=False)
        observed = singular > singular[0]*.12
        rank = int(observed.sum())
        # The FK initialization is a weak prior, particularly useful for a
        # cylinder's rotation about its axis. Do not integrate tiny noisy
        # updates into a large, unsupported correction over many iterations.
        prior = np.r_[Rotation.from_matrix(total[:3, :3]).as_rotvec()*scale,
                      total[:3, :3]@center+total[:3, 3]-center]
        damping = len(p)*.003
        projected = singular*(u.T @ (residual*np.sqrt(weights))) + damping*(vt@prior)
        step = -vt[observed].T @ (projected[observed]/(singular[observed]**2+damping))
        step -= vt[~observed].T @ (vt[~observed]@prior)
        rotation_vector, translation = step[:3]/scale, step[3:]
        if np.linalg.norm(rotation_vector) > np.radians(5):
            rotation_vector *= np.radians(5)/np.linalg.norm(rotation_vector)
        if np.linalg.norm(translation) > .015:
            translation *= .015/np.linalg.norm(translation)
        rotation = Rotation.from_rotvec(rotation_vector).as_matrix()
        delta = np.eye(4)
        delta[:3, :3], delta[:3, 3] = rotation, center+translation-rotation@center
        total = delta @ total
        if np.linalg.norm(translation) < 1e-6 and np.linalg.norm(rotation_vector) < 1e-5:
            break
    current = transform(source, total)
    distance, indices = tree.query(current)
    normal_match = np.abs(np.sum((source_normal @ total[:3, :3].T)*normal[indices], axis=1)) > .85
    close = (distance < residual_limit*1.5) & source_ok & target_ok[indices] & normal_match
    residual = np.abs(np.sum((current-target[indices])*normal[indices], axis=1))
    error = float(np.percentile(residual[close], 90)) if close.sum() >= 60 else float('inf')
    correction_m = float(np.linalg.norm(total[:3, 3]))
    correction_deg = float(np.degrees(Rotation.from_matrix(total[:3, :3]).magnitude()))
    accepted = bool(close.mean() >= min_overlap and error <= residual_limit and correction_m <= .04 and correction_deg <= 15.)
    return {'accepted': accepted, 'refined': accepted and rank == 6, 'transform': total if accepted else identity,
            'reason': ('registered' if rank == 6 else 'underconstrained_overlap') if accepted else 'registration_quality',
            'overlap': float(close.mean()), 'residual_p90_m': error, 'iterations': iteration+1,
            'observable_rank': rank, 'correction_m': correction_m, 'correction_deg': correction_deg}


class ScanMap:
    def __init__(self, *, voxel_m=.003, max_views=24, max_voxels=50_000):
        self.cloud = VoxelCloud(voxel_m=voxel_m, max_voxels=max_voxels)
        self.reference_pose = None
        self.identity = None
        self.last_seq = -1
        self.accepted_views = 0
        self.max_views = max_views
        self.voxel_m = voxel_m
        self.last_pose = None
        self.last_relative = None

    def points(self):
        return self.cloud.points()

    def add(self, points_camera, nominal_pose, *, identity, seq):
        points = _points(points_camera)
        pose = camera_transform(nominal_pose)
        if (len(identity) != 3 or any(not isinstance(v, str) or not v for v in identity)
                or type(seq) is not int or seq < 0):
            raise ValueError('invalid scan identity or sequence')
        if self.identity is not None and (identity != self.identity or seq <= self.last_seq):
            return {'accepted': False, 'reason': 'identity_or_sequence', 'new_voxels': 0}
        if self.accepted_views >= self.max_views or len(points) < 60:
            return {'accepted': False, 'reason': 'view_budget_or_points', 'new_voxels': 0}
        if self.reference_pose is None:
            self.reference_pose, self.identity = pose.copy(), identity
            relative, aligned = np.eye(4), points.copy()
            registration = {'accepted': True, 'refined': False, 'reason': 'first_optical_anchor',
                            'overlap': 1., 'residual_p90_m': 0., 'transform': np.eye(4)}
        else:
            relative = np.linalg.inv(self.reference_pose) @ pose
            placed = transform(points, relative)
            if self.last_pose is not None and np.max(np.abs(pose-self.last_pose)) < 1e-9:
                relative = self.last_relative.copy()
                placed = transform(points, relative)
                distance = cKDTree(self.points()).query(placed)[0]
                inlier = distance < .025
                error = float(np.percentile(distance[inlier], 90)) if np.any(inlier) else float('inf')
                registration = {'accepted': bool(inlier.mean() >= .2 and error <= .0045),
                                'refined': False, 'reason': 'repeated_pose', 'transform': np.eye(4),
                                'overlap': float(inlier.mean()), 'residual_p90_m': error}
            else:
                registration = register_overlap(placed, self.points())
            if not registration['accepted']:
                return {**registration, 'new_voxels': 0}
            relative = registration['transform'] @ relative
            aligned = transform(points, relative)
        before = len(self.points())
        self.cloud.add(aligned)
        after = len(self.points())
        self.last_seq = seq
        self.last_pose, self.last_relative = pose.copy(), relative.copy()
        self.accepted_views += 1
        return {**registration, 'new_voxels': after-before, 'map_points': after,
                'camera_in_reference': relative, 'aligned_points': aligned,
                'new_fraction': (after-before)/max(before, 1), 'accepted_views': self.accepted_views}


class ViewPlanner:
    """Reachable FK view proposals from observations, without mesh/truth access."""
    def __init__(self, camera_model, *, candidates=12000, seed=41):
        if not 100 <= candidates <= 30_000:
            raise ValueError('invalid view candidate budget')
        self.model = camera_model
        self.q = qmc.scale(qmc.Halton(d=4, scramble=True, seed=seed).random(candidates), camera_model.lower, camera_model.upper)
        self.poses = np.array([camera_model.camera_pose(q) for q in self.q])

    def choose(self, observed_base, current_q, accepted_poses, attempted_q):
        points = _points(observed_base)
        if len(points) < 3 or not accepted_poses:
            return None
        center = (np.percentile(points, 5, axis=0) + np.percentile(points, 95, axis=0))/2
        origin = self.poses[:, :3, 3]
        vector = center-origin
        camera = np.einsum('nji,nj->ni', self.poses[:, :3, :3], vector)
        distance = np.linalg.norm(vector, axis=1)
        normalized = np.abs(camera[:, :2])/np.maximum(camera[:, 2:3], 1e-6)
        usable = (camera[:, 2] > .16) & (camera[:, 2] < .8) & (normalized[:, 0] < .50) & (normalized[:, 1] < .38)
        direction = -vector/np.maximum(distance[:, None], 1e-9)
        seen = np.array([pose[:3, 3]-center for pose in accepted_poses])
        seen /= np.linalg.norm(seen, axis=1)[:, None]
        angle = np.degrees(np.arccos(np.clip(direction @ seen.T, -1, 1)))
        novelty = angle.min(axis=1)
        usable &= (novelty >= 4.) & (novelty <= 42.)
        span = self.model.upper-self.model.lower
        tried = np.asarray(attempted_q)
        for q in tried:
            usable &= np.linalg.norm((self.q-q)/span, axis=1) > .025
        # Keep all currently observed extent inside the predicted frame. The
        # unknown surface is not invented to score candidate visibility.
        corners = np.array([[x,y,z] for x in np.percentile(points, [2,98], axis=0)[:,0]
                            for y in np.percentile(points, [2,98], axis=0)[:,1]
                            for z in np.percentile(points, [2,98], axis=0)[:,2]])
        candidate_ids = np.nonzero(usable)[0]
        for j in candidate_ids:
            p = (corners-origin[j]) @ self.poses[j, :3, :3]
            uv = np.abs(p[:, :2])/np.maximum(p[:, 2:3], 1e-6)
            if np.any(p[:, 2] < .10) or np.any(uv[:, 0] > .73) or np.any(uv[:, 1] > .54):
                usable[j] = False
        ids = np.nonzero(usable)[0]
        if not len(ids):
            return None
        motion = np.linalg.norm((self.q-np.asarray(current_q))/span, axis=1)
        score = novelty - 8*motion - 5*np.linalg.norm(normalized, axis=1)
        chosen = ids[np.argmax(score[ids])]
        return {'q': self.q[chosen].copy(), 'pose': self.poses[chosen].copy(),
                'novelty_deg': float(novelty[chosen]), 'nearest_accepted_view_deg': float(novelty[chosen]),
                'feasible_candidates': len(ids), 'score': float(score[chosen])}


def refine_joint_scan(model, frames, *, max_frames=24):
    """Fit shared bend zero offsets using observed inter-view surface overlap.

    This optional batch stage assumes a static object/base and constant bend
    offsets during the scan. Linear and roll zero shifts are gauge freedoms
    for these relative camera poses and are deliberately not estimated. It
    neither commands corrected joints nor identifies absolute world placement.
    Independent per-view pose corrections are not composed into this result.
    """
    if not 4 <= len(frames) <= min(max_frames, 24):
        return {'accepted': False, 'reason': 'joint_fit_frame_budget'}
    q = np.array([item[0] for item in frames], dtype=float)
    if q.shape != (len(frames), 4) or not np.all(np.isfinite(q)) or np.any(q < model.lower) or np.any(q > model.upper):
        raise ValueError('invalid scan joint observations')
    points = []
    normals, good, trees = [], [], []
    for _, raw in frames:
        p = voxel_points(raw, .004)
        if len(p) < 60:
            return {'accepted': False, 'reason': 'joint_fit_points'}
        if len(p) > 1200:
            p = p[np.linspace(0, len(p)-1, 1200, dtype=int)]
        n, ok = _surface_normals(p)
        points.append(p); normals.append(n); good.append(ok); trees.append(cKDTree(p))

    def poses(offset):
        bias = np.r_[0., 0., offset]
        return np.array([model.camera_pose(np.clip(value+bias, model.lower, model.upper)) for value in q])

    def correspondences(ts, max_distance):
        pairs = []
        for i in range(len(q)):
            for j in range(i):
                relative = np.linalg.inv(ts[j])@ts[i]
                placed = transform(points[i], relative)
                distance, indices = trees[j].query(placed)
                same = np.abs(np.sum((normals[i]@relative[:3, :3].T)*normals[j][indices], axis=1)) > .85
                ids = np.nonzero((distance < max_distance) & same & good[i] & good[j][indices])[0]
                if len(ids) < 60:
                    continue
                fraction = len(ids)/len(points[i])
                if len(ids) > 180:
                    ids = ids[np.linspace(0, len(ids)-1, 180, dtype=int)]
                pairs.append((i, j, points[i][ids], points[j][indices[ids]], normals[j][indices[ids]], fraction))
        return pairs

    offset = np.zeros(2)
    cost_before = cost_after = None
    for iteration in range(5):
        pairs = correspondences(poses(offset), .018)
        if len(pairs) < 3:
            return {'accepted': False, 'reason': 'joint_fit_overlap'}

        def residual(value):
            ts = poses(value)
            values = [np.sum((transform(src, np.linalg.inv(ts[j])@ts[i])-dst)*n, axis=1)
                      for i, j, src, dst, n, _ in pairs]
            return np.r_[np.concatenate(values), value*.003]

        fit = least_squares(residual, offset, bounds=(-.04, .04), diff_step=1e-3,
                            loss='huber', f_scale=.002, max_nfev=30, xtol=1e-6, ftol=1e-6)
        if cost_before is None:
            cost_before = float(np.mean(residual(offset)[:-2]**2))
        offset = fit.x
        cost_after = float(np.mean(residual(offset)[:-2]**2))
    singular = np.linalg.svd(fit.jac[:-2], compute_uv=False)
    if len(singular) < 2 or singular[0] < .01 or singular[-1] < singular[0]*.02 or np.any(np.abs(offset) > .0395):
        return {'accepted': False, 'reason': 'joint_fit_unobservable_or_bound'}
    ts = poses(offset)
    pairs = correspondences(ts, .008)
    connected = {0}; edges = []
    for i, j, src, dst, n, fraction in pairs:
        residual = np.abs(np.sum((transform(src, np.linalg.inv(ts[j])@ts[i])-dst)*n, axis=1))
        if fraction >= .20 and np.percentile(residual, 90) <= .0045:
            edges.append((i, j))
    for _ in frames:
        for i, j in edges:
            if i in connected or j in connected:
                connected.update((i, j))
    if len(connected) < 4:
        return {'accepted': False, 'reason': 'joint_fit_disconnected'}
    relatives = np.linalg.inv(ts[0])@ts
    cloud = VoxelCloud(voxel_m=.003, max_voxels=50_000)
    for i in sorted(connected):
        cloud.add(transform(frames[i][1], relatives[i]))
    return {'accepted': True, 'reason': 'shared_bend_offset_fit', 'bend_offsets_rad': offset.tolist(),
            'used_frames': sorted(connected), 'constraint_edges': edges, 'iterations': iteration+1,
            'surface_mse_before': cost_before, 'surface_mse_after': cost_after,
            'jacobian_singular_values': singular.tolist(), 'camera_in_reference': relatives,
            'points': cloud.points(), 'absolute_world_placement_observed': False}
