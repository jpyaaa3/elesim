"""Reachable SEE frontier views, validated with EleSim's actual camera and arm."""
from __future__ import annotations

import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from .multiview import ViewPlanner, transform
from .see import SeeBackend, SeeOptions
from ..robot.arm.approach import JointApproach


class SeeViewPlanner(ViewPlanner):
    def __init__(self, *args, backend=None, return_q=None, fallback=False, **kwargs):
        # Cached reachable FK poses seed constrained refinement; they do not
        # define the information objective or command unvalidated motion.
        kwargs.setdefault('candidates', 1024)
        super().__init__(*args, **kwargs)
        self.policy = 'see'
        self.first_q = None
        self.fallback = fallback
        self.information = None
        # Explicit experimental calibration for the scan's capped 20k-point
        # observations. Upstream's generic density remains available in SeeBackend.
        self.backend = backend if backend is not None else SeeBackend(
            options=SeeOptions(density_per_m3=5_000_000))
        self.frames = []
        self.intrinsics = None
        self.return_q = None if return_q is None else np.array(return_q, float, copy=True)
        if self.return_q is not None and (self.return_q.shape != (4,) or
                not np.isfinite(self.return_q).all() or
                np.any(self.return_q < self.operating_bounds[:, 0]) or
                np.any(self.return_q > self.operating_bounds[:, 1])):
            raise ValueError('invalid SEE return posture')

    def set_observations(self, frames, intrinsics):
        # Snapshot is rebuilt after registration changes: no stale poses and no
        # repeated feeding of fused points as new camera measurements.
        self.frames = [(transform(points, pose), np.array(pose, copy=True))
                       for points, pose in frames]
        self.intrinsics = np.array([intrinsics.fx, intrinsics.fy, intrinsics.cx,
                                   intrinsics.cy, intrinsics.width, intrinsics.height])

    def _visible(self, points, pose, blockers, *, cancelled=lambda: False):
        local = (points-pose[:3, 3]) @ pose[:3, :3]
        fx, fy, cx, cy, width, height = self.intrinsics
        uv = local[:, :2]/np.maximum(local[:, 2:3], 1e-9)*[fx, fy]+[cx, cy]
        visible = ((local[:, 2] > .16) & (local[:, 2] < .8) &
                   (uv[:, 0] >= 4) & (uv[:, 0] < width-4) &
                   (uv[:, 1] >= 4) & (uv[:, 1] < height-4))
        for i in np.flatnonzero(visible):
            if cancelled():
                raise InterruptedError('SEE visibility cancelled')
            direction = points[i]-pose[:3, 3]
            distance = np.linalg.norm(direction)
            ray = pose[:3, 3]+np.arange(.02, max(.02, distance-.02), .006)[:, None]*direction/distance
            if len(ray) and np.any(blockers.query(ray)[0] < .006):
                visible[i] = False
        return visible

    def _contains_extent(self, points, pose):
        points = np.asarray(points, float)
        if self.roi_height_m is not None:
            center = np.mean(np.percentile(points[:, 2], [5, 95]))
            roi = points[abs(points[:, 2]-center) <= self.roi_height_m/2]
            if len(roi) >= 3:
                points = roi
        lo, hi = np.percentile(points, [2, 98], axis=0)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                            for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        local = (corners-pose[:3, 3]) @ pose[:3, :3]
        fx, fy, cx, cy, width, height = self.intrinsics
        uv = local[:, :2]/np.maximum(local[:, 2:3], 1e-9)*[fx, fy]+[cx, cy]
        return bool(np.all((local[:, 2] > .1) & (local[:, 2] < .8)) and
                    np.all((uv >= [4, 4]) & (uv < [width-4, height-4])))

    def _return(self, q):
        # The first observation is a usable bridge to the parent's recorded
        # approach. The parent will re-certify its complete route to Home.
        anchor = self.first_q if self.return_q is None else self.return_q
        certificate = self.collision_checker.certify_segment(q, anchor)
        if certificate['accepted']:
            return certificate, float(np.max(abs(q-anchor)/JointApproach.rates)), 'preferred'
        if np.array_equal(anchor, self.first_q):
            return None
        certificate = self.collision_checker.certify_segment(q, self.first_q)
        if not certificate['accepted']:
            return None
        time_to_first = np.max(abs(q-self.first_q)/JointApproach.rates)
        onward = np.max(abs(self.first_q-anchor)/JointApproach.rates)
        return certificate, float(time_to_first+onward), 'via_first'

    def choose(self, observed_base, current_q, accepted_poses, attempted_q, *, cancelled=lambda: False):
        if self.first_q is None:
            self.first_q = np.array(current_q, dtype=float, copy=True)
        choice = self._choose_see(observed_base, current_q, accepted_poses, attempted_q,
                                  cancelled=cancelled)
        if choice is not None or not self.fallback:
            return choice
        if self.information is None:
            self.information = ViewPlanner(self.model, collision_checker=self.collision_checker,
                operating_bounds=self.operating_bounds, roi_height_m=self.roi_height_m,
                cancelled=cancelled)
        choice = self.information.choose(observed_base, current_q, accepted_poses, attempted_q,
            cancelled=cancelled, can_return=lambda q: self._return(q) is not None)
        self.last_search['fallback'] = dict(self.information.last_search)
        if choice is not None:
            choice['policy'] = 'information'
        return choice

    def frontiers(self, observed_base, *, cancelled):
        if not self.frames or self.intrinsics is None:
            raise RuntimeError('SEE requires coherent accepted observations')
        proposals = self.backend.propose(self.frames, self.intrinsics, cancelled=cancelled)
        self.last_search = dict(policy='see', proposals=len(proposals), unsafe_endpoints=0,
                                unsafe_paths=0, return_rejections=0)
        if not len(proposals):
            self.last_search['reason'] = 'frontier_exhausted_not_completeness'
            return None
        proposals = np.asarray(proposals, float)
        if proposals.ndim != 2 or proposals.shape[1] != 9 or not np.isfinite(proposals).all():
            raise ValueError('invalid SEE proposals')
        center = np.mean(np.percentile(observed_base, [5, 95], axis=0), axis=0)
        if self.roi_height_m is not None:
            proposals = proposals[abs(proposals[:, 8]-center[2]) <= self.roi_height_m/2]
        self.last_search['roi_proposals'] = len(proposals)
        if not len(proposals):
            self.last_search['reason'] = 'no_frontiers_in_task_roi'
            return None
        return proposals

    def _choose_see(self, observed_base, current_q, accepted_poses, attempted_q, *, cancelled):
        proposals = self.frontiers(observed_base, cancelled=cancelled)
        if proposals is None:
            return None
        current_q = np.asarray(current_q, float)
        rates = JointApproach.rates
        center = np.mean(np.percentile(observed_base, [5, 95], axis=0), axis=0)
        lower, upper = self.operating_bounds.T
        active = upper-lower > 1e-9
        candidates = []
        def add_candidate(q):
            if any(np.all(abs(q-tried) < [.002, .03, .03, .03]) for tried in attempted_q):
                return
            if not any(np.all(abs(q-old) < [.002, .03, .03, .03]) for old in candidates):
                candidates.append(q.copy())
        start = time.monotonic()
        # Bounded local refinement from achievable FK seeds, nearest proposals
        # first. Exact five-dimensional camera pose is not imposed on four DOFs.
        near = np.argsort(np.linalg.norm(proposals[:, :3]-self.model.camera_pose(current_q)[:3, 3], axis=1))[:32]
        for i in near:
            if cancelled():
                raise InterruptedError('SEE reachability cancelled')
            if time.monotonic()-start > 3.:
                break
            desired, direction, frontier = np.split(proposals[i], 3)
            seed_cost = np.linalg.norm(self.poses[:, :3, 3]-desired, axis=1)
            seed_cost += .15*(1-self.poses[:, :3, 2]@direction)
            for seed in [current_q.copy(), *self.q[np.argsort(seed_cost)[:2]]]:
                seed = seed.copy()
                add_candidate(seed)
                def unpack(x):
                    q = seed.copy(); q[active] = x
                    return q
                def residual(x):
                    if cancelled():
                        raise InterruptedError('SEE reachability cancelled')
                    pose = self.model.camera_pose(unpack(x))
                    to_front = frontier-pose[:3, 3]
                    to_front /= max(np.linalg.norm(to_front), 1e-9)
                    return np.r_[(pose[:3, 3]-desired)/.15,
                                 (pose[:3, 2]-to_front)*2]
                q = seed if not active.any() else unpack(least_squares(residual, seed[active],
                    bounds=(lower[active], upper[active]), max_nfev=24).x)
                add_candidate(q)
                # Retain reachable intermediate projections instead of forcing
                # an unconstrained SEE camera location through the robot body.
                add_candidate(current_q+.5*(q-current_q))
        # Fused measured cloud for occlusion, augmented by measured scene obstacles.
        blockers = np.asarray(observed_base)
        scene = getattr(self.collision_checker, 'cloud', None)
        if scene is not None:
            scene = scene.points() if hasattr(scene, 'points') else np.asarray(scene)
            if len(scene):
                blockers = np.concatenate([blockers, scene])
        tree = cKDTree(blockers)
        targets = proposals[:, 6:9]
        best = None
        seen = np.array([pose[:3, 3]-center for pose in accepted_poses])
        seen /= np.maximum(np.linalg.norm(seen, axis=1)[:, None], 1e-9)
        self.last_search.update(extent_rejections=0, novelty_rejections=0, visibility_rejections=0)
        rank_started = time.monotonic()
        for q in candidates:
            if time.monotonic()-rank_started > 5.:
                self.last_search['ranking_budget_exhausted'] = True
                break
            if cancelled():
                raise InterruptedError('SEE view selection cancelled')
            if not self.collision_checker.configuration_free(q):
                self.last_search['unsafe_endpoints'] += 1; continue
            pose = self.model.camera_pose(q)
            direction = pose[:3, 3]-center
            direction /= max(np.linalg.norm(direction), 1e-9)
            novelty = np.degrees(np.arccos(np.clip(seen@direction, -1, 1))).min()
            if not 8. <= novelty <= 42.:
                self.last_search['novelty_rejections'] += 1
                continue
            if not self._contains_extent(observed_base, pose):
                self.last_search['extent_rejections'] += 1
                continue
            visible = self._visible(targets, pose, tree, cancelled=cancelled)
            toward = targets-pose[:3, 3]
            toward /= np.maximum(np.linalg.norm(toward, axis=1)[:, None], 1e-9)
            visible &= np.sum(toward*proposals[:, 3:6], axis=1) > .15
            if not visible.any():
                self.last_search['visibility_rejections'] += 1
                continue
            certificate = self.collision_checker.certify_segment(current_q, q)
            if not certificate['accepted']:
                self.last_search['unsafe_paths'] += 1; continue
            route = self._return(q)
            if route is None:
                self.last_search['return_rejections'] += 1; continue
            return_certificate, return_time, return_mode = route
            travel = float(np.max(abs(q-current_q)/rates))
            score = float(visible.sum())/(1.+travel+return_time)
            if best is None or score > best['score']:
                best = dict(q=q, pose=pose, path_certificate=certificate,
                    return_certificate=return_certificate, return_mode=return_mode, score=score,
                    expected_visibility_gain=float(visible.mean()),
                    visible_frontiers=int(visible.sum()), travel_s=travel,
                    return_s=return_time, feasible_candidates=len(candidates), policy='see')
            time.sleep(.001)
        self.last_search['reachable_candidates'] = len(candidates)
        self.last_search['reason'] = 'selected' if best else 'no_certified_frontier_view'
        if best:
            self.last_search.update({k: best[k] for k in ('visible_frontiers', 'travel_s', 'return_s', 'return_mode')})
        return best
