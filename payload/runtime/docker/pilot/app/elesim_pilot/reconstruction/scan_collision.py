"""Arm self-collision plus conservative separation from observed RGB-D surfaces.

This checks modeled spheres and measured surfaces. Unknown/occluded geometry,
GO2, cables and dynamics are not certified; the runtime entrypoint is Sim-only.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from elesim_pilot.vision.partial_cloud import VoxelCloud, reconstruct_view


class ScanCollision:
    def __init__(self, arm, base_world_transform, *, margin_m=.015):
        self.arm = arm
        self.base = np.array(base_world_transform, dtype=float, copy=True)
        self.margin_m = margin_m
        self.cloud = VoxelCloud(voxel_m=.01, max_voxels=50_000)
        self.tree = None
        self._radii = np.concatenate([arm.radii[name] for name in arm.names])
        lipschitz = []
        for name in arm.names:
            bound = np.zeros(4)
            reach = float(np.max(np.linalg.norm(arm.local[name], axis=1)))
            for index in reversed(arm.ancestry[name]):
                joint = arm.joints[index]
                axis = joint['index']
                if axis is not None:
                    bound[axis] += abs(joint['multiplier'])*(1. if joint['kind'] == 'prismatic' else reach)
                reach += float(np.linalg.norm(joint['t'][:3, 3]))
                if joint['kind'] == 'prismatic':
                    reach += float(np.max(np.abs(arm.bounds[axis])))
            lipschitz.extend([bound]*len(arm.radii[name]))
        self._lipschitz = np.array(lipschitz)
        self._tracking_margin = self._lipschitz@np.asarray(arm.uncertainty)

    def __getattr__(self, name):
        return getattr(self.arm, name)

    def _centers(self, q):
        poses = self.arm.link_poses(q)
        local = np.concatenate([self.arm.local[name]@poses[name][:3, :3].T+poses[name][:3, 3]
                                for name in self.arm.names])
        return local@self.base[:3, :3].T+self.base[:3, 3]

    def observe(self, measurement, world_from_camera):
        sensor = measurement.sensor
        view = reconstruct_view(sensor, np.ones(sensor.depth_raw.shape, bool), world_from_camera,
            max_points=6000)
        centers = self._centers(sensor.encoder_q)
        points = view.points_before
        # Remove the modeled arm from its own depth observations. Chunking
        # bounds temporary memory independently of the scene resolution.
        kept = []
        for start in range(0, len(points), 256):
            block = points[start:start+256]
            gap = np.linalg.norm(block[:, None, :]-centers[None, :, :], axis=2)-self._radii
            kept.append(block[np.all(gap > self.margin_m, axis=1)])
        if kept:
            self.cloud.add(np.concatenate(kept))
        fused = self.cloud.points()
        self.tree = cKDTree(fused) if len(fused) else None

    def _gaps(self, q):
        if self.tree is None:
            return np.full(len(self._radii), np.inf)
        return self.tree.query(self._centers(q))[0]-self._radii-self.margin_m-self._tracking_margin

    def configuration_free(self, q):
        return self.arm.configuration_free(q) and bool(np.all(self._gaps(q) > 0))

    def certify_segment(self, start, goal, *, max_checks=512):
        own = self.arm.certify_segment(start, goal)
        if not own['accepted']:
            return own
        if self.tree is None:
            return {**own, 'environment_scope': 'not_observed_yet'}
        start, goal = np.array(start), np.array(goal)
        left, right = self._gaps(start), self._gaps(goal)
        pending = [(start, goal, left, right)]
        checks = 2
        if min(left.min(), right.min()) <= 0:
            return {'accepted': False, 'reason': 'observed_surface_collision'}
        while pending:
            a, b, ca, cb = pending.pop()
            if np.all(np.minimum(ca, cb)-self._lipschitz@np.abs(b-a)*.5 > 0):
                continue
            if checks >= max_checks:
                return {'accepted': False, 'reason': 'observed_surface_certificate_budget'}
            mid = (a+b)*.5
            cm = self._gaps(mid)
            checks += 1
            if np.any(cm <= 0):
                return {'accepted': False, 'reason': 'observed_surface_collision_on_path'}
            pending.extend(((a, mid, ca, cm), (mid, b, cm, cb)))
        return {**own, 'environment_scope': 'observed_surfaces_only', 'environment_checks': checks}
