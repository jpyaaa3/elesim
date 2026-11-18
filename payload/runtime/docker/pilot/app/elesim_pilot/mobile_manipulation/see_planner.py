"""SEE camera goals solved jointly over measured planar base and arm coordinates."""
from dataclasses import dataclass
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from elesim_pilot.robot.arm.approach import JointApproach
from elesim_pilot.vision.see_planner import SeeViewPlanner
from elesim_pilot.vision.multiview import ViewPlanner
from elesim_pilot.reconstruction.scan_collision import ScanCollision
from .base_planner import body_frame, FloorObservationRequired
from .contracts import BaseGoal, angle
from .local_map import transport_footprint


@dataclass(frozen=True)
class ViewMotion:
    base: BaseGoal
    start_q: np.ndarray
    footprint: np.ndarray
    certificate: dict
    start_arm_base: np.ndarray
    travel_s: float
    reach_m: float = 0.


class MobileSeePlanner(SeeViewPlanner):
    """A single next-view objective for both stationary and moving-base views.

    The local base box bounds each planning step, not the object's presumed
    location. Only measured surfaces produce SEE proposals or free floor cells.
    """

    def __init__(self, model, ports, mobility, **options):
        options['collision_checker'] = ports.collision
        super().__init__(model, fallback=False, return_q=ports.model.meta['home'], **options)
        self.ports, self.mobility = ports, mobility
        self.local_poses = np.linalg.inv(self.collision_checker.base)[None] @ self.poses

    def update_base(self, snapshot):
        p = self.ports
        base = p.service._go2_arm_mount.arm_base_world_transform(
            snapshot['base']['pos'], snapshot['base']['rpy'])
        self.model.context['base_world_T'] = base.copy()
        self.collision_checker.base = base.copy()
        self.poses = base[None] @ self.local_poses
        return base

    def camera_at(self, pose, q, snapshot):
        p = self.ports
        base = body_frame(pose, snapshot['base']['pos'][2], snapshot['base']['rpy'][:2]) @ p.arm_in_body
        local = np.linalg.inv(self.collision_checker.base) @ self.model.camera_pose(q)
        return base @ local

    def motion(self, base_pose, q, snapshot, *, cancelled):
        """Certify all arm shapes independently of base/arm execution timing.

        Intermediate joint shapes are enclosed by sphere-motion bounds before
        projection to the floor map. A free swept footprint therefore supports
        concurrent, feedback-paced base and arm movement, including either
        actuator finishing first.
        """
        p = self.ports
        start_q = snapshot['joint']['q']
        current = snapshot['pose']
        moved = np.linalg.norm(base_pose[:2]-current[:2]) > 1e-6 or abs(angle(base_pose[2]-current[2])) > 1e-6
        checker = ScanCollision(p.arm, self.collision_checker.base, reserve_tracking=True)
        checker.cloud, checker.tree = p.collision.cloud, p.collision.tree
        own = checker.arm.certify_segment(start_q, q)
        if not own['accepted']:
            return None
        if not moved:
            certificate = checker.certify_segment(start_q, q)
            if not certificate['accepted']:
                return None
            path = current[None].copy()
            footprint = p.footprint
            base_time = 0.
            reach = 0.
        else:
            delta = abs(np.asarray(q)-start_q)
            displacement = checker._lipschitz @ delta
            count = max(1, int(np.ceil(float(displacement.max())/.01)))
            if count > 128:
                return None
            inverse_body = np.linalg.inv(p._measured_body(snapshot))
            centers = np.concatenate([checker._centers(start_q+(q-start_q)*f)
                                      for f in np.linspace(0., 1., count+1)])
            reach = float(np.max(np.linalg.norm(centers-checker.base[:3,3],axis=1))+checker._radii.max())
            centers = centers @ inverse_body[:3, :3].T + inverse_body[:3, 3]
            # Every shape between adjacent samples lies inside these enlarged
            # spheres, including the existing joint tracking allowance.
            radii = np.tile(checker._radii+checker._tracking_margin+
                            displacement/(2*count), count+1)
            footprint = transport_footprint(centers, radii, spacing=p.grid.resolution/2)
            radius = float(np.max(np.linalg.norm(footprint, axis=1)))
            margin = p.s.base_position_tolerance_m+2*radius*np.sin(p.s.base_yaw_tolerance_rad/2)
            for position in (current, base_pose):
                status = p.grid.footprint_status(position, footprint, margin_m=margin)
                if status['blocked_cells']:
                    return None
                if status['unknown_cells']:
                    raise FloorObservationRequired(position,footprint=footprint,margin_m=margin)
            try:
                path = p.grid.path(current, base_pose, footprint, margin_m=margin, max_expansions=512)
            except ValueError:
                difference=base_pose-current;difference[2]=angle(difference[2])
                for fraction in np.linspace(0.,1.,17):
                    position=current+difference*fraction
                    status=p.grid.footprint_status(position,footprint,margin_m=margin)
                    if status['unknown_cells'] and not status['blocked_cells']:
                        raise FloorObservationRequired(position,footprint=footprint,margin_m=margin)
                return None
            if cancelled():
                raise InterruptedError('mobile SEE path cancelled')
            difference = np.diff(path, axis=0)
            difference[:, 2] = angle(difference[:, 2])
            base_time = float(np.sum(np.maximum(np.linalg.norm(difference[:, :2], axis=1)/p.s.base_speed_mps,
                                               abs(difference[:, 2])/p.s.yaw_speed_radps)))
            certificate = {**own, 'environment_scope': 'observed_swept_floor',
                           'swept_joint_samples': count+1, 'map_revision': p.grid.revision}
        goal_base = body_frame(base_pose, snapshot['base']['pos'][2], snapshot['base']['rpy'][:2]) @ p.arm_in_body
        checker.base = goal_base
        if not checker.configuration_free(q):
            return None
        if not checker.certify_segment(q, self.return_q)['accepted']:
            return None
        arm_time = float(np.max(abs(q-start_q)/JointApproach.rates))
        goal = BaseGoal(base_pose, path, q, dict(kind='observation'))
        return ViewMotion(goal, start_q.copy(), footprint, certificate,
                          self.collision_checker.base.copy(), max(base_time, arm_time),reach)

    def choose(self, observed_base, current_q, accepted_poses, attempted_q, *, cancelled=lambda: False):
        proposals = self.frontiers(observed_base, cancelled=cancelled)
        if proposals is None:
            return self._information_view(observed_base, accepted_poses, cancelled=cancelled)
        while True:
            snapshot = self.ports.feedback.snapshot(self.ports.route()[:2])
            self.update_base(snapshot)
            choice, missing = self._choose_mobile(proposals, observed_base, snapshot, accepted_poses,
                                                   cancelled=cancelled)
            if choice is not None:
                return choice
            if missing is None:
                return self._information_view(observed_base, accepted_poses, cancelled=cancelled)
            # Floor recovery preserves this scan's accepted measurements and
            # SEE frontiers. It neither changes the target nor restarts a scan.
            self.mobility.observe_floor(missing, cancelled=cancelled)

    def _information_view(self, observed, accepted, *, cancelled):
        snapshot = self.ports.feedback.snapshot(self.ports.route()[:2])
        self.update_base(snapshot)
        if self.information is None:
            self.information = ViewPlanner(self.model, collision_checker=self.collision_checker,
                candidates=1024, operating_bounds=self.operating_bounds, roi_height_m=self.roi_height_m,
                cancelled=cancelled)
        self.information.poses = self.poses
        self.information.q = self.q
        choice = self.information.choose(observed, snapshot['joint']['q'], accepted, [], cancelled=cancelled,
            can_return=lambda q: self.collision_checker.certify_segment(q, self.return_q)['accepted'])
        self.last_search['fallback'] = self.information.last_search.copy()
        if choice is None:
            return None
        motion = self.motion(snapshot['pose'], choice['q'], snapshot, cancelled=cancelled)
        if motion is None:
            return None
        return {**choice, 'policy': 'information', 'base_pose': snapshot['pose'].copy(), 'motion': motion}

    def _choose_mobile(self, proposals, observed, snapshot, accepted, *, cancelled):
        p = self.ports
        current, current_q = snapshot['pose'], snapshot['joint']['q']
        lower, upper = self.operating_bounds.T
        center = np.mean(np.percentile(observed, [5, 95], axis=0), axis=0)
        blockers = np.concatenate([observed, p.collision.cloud.points()])
        tree = cKDTree(blockers)
        seen = np.array([pose[:3, 3]-center for pose in accepted])
        seen /= np.maximum(np.linalg.norm(seen, axis=1)[:, None], 1e-9)
        order = np.argsort(np.linalg.norm(proposals[:, :3]-self.model.camera_pose(current_q)[:3, 3], axis=1))[:16]
        candidates = []
        started = time.monotonic()
        evaluations = 0
        for index in order:
            if cancelled():
                raise InterruptedError('mobile SEE IK cancelled')
            if time.monotonic()-started > 5.:
                break
            desired, _, frontier = np.split(proposals[index], 3)
            seed_cost = np.linalg.norm(self.poses[:, :3, 3]-desired, axis=1)
            for seed in (current_q, self.q[np.argmin(seed_cost)]):
                # Fixed-base and coupled solutions compete on the same SEE
                # score, with no separate observation-position stage.
                for moving in (False, True):
                    start = np.r_[np.zeros(3), seed]
                    low = np.r_[[-.25, -.25, -np.pi/6], lower]
                    high = np.r_[[.25, .25, np.pi/6], upper]
                    active = high-low > 1e-9
                    if not moving:
                        active[:3] = False
                    def unpack(value):
                        full = start.copy()
                        full[active] = value
                        return current+full[:3], full[3:]
                    def residual(value):
                        nonlocal evaluations
                        if cancelled():
                            raise InterruptedError('mobile SEE IK cancelled')
                        evaluations += 1
                        if evaluations % 16 == 0:
                            time.sleep(.001)
                        base_pose, q = unpack(value)
                        camera = self.camera_at(base_pose, q, snapshot)
                        ray = frontier-camera[:3, 3]
                        ray /= max(np.linalg.norm(ray), 1e-9)
                        return np.r_[(camera[:3, 3]-desired)/.15,
                                     2*(camera[:3, 2]-ray), .08*(base_pose-current)]
                    value = start[active]
                    if active.any():
                        value = least_squares(residual, value,
                            bounds=(low[active], high[active]), max_nfev=24).x
                    base_pose, q = unpack(value)
                    if any(np.linalg.norm(base_pose-old[0])<.005 and np.max(abs(q-old[1]))<.01
                           for old in candidates):
                        continue
                    candidates.append((base_pose, q))
        best, missing = None, None
        self.last_search.update(mobile_candidates=len(candidates), unknown_floor=0,
                                rejected_views=0, rejected_paths=0)
        ranked = []
        for base_pose, q in candidates:
            if cancelled():
                raise InterruptedError('mobile SEE ranking cancelled')
            pose = self.camera_at(base_pose, q, snapshot)
            direction = pose[:3, 3]-center
            direction /= max(np.linalg.norm(direction), 1e-9)
            novelty = np.degrees(np.arccos(np.clip(seen @ direction, -1., 1.))).min()
            if not 8. <= novelty <= 42. or not self._contains_extent(observed, pose):
                self.last_search['rejected_views'] += 1
                continue
            if any(np.linalg.norm(pose[:3, 3]-old[:3, 3])<.01 and
                   pose[:3, 2] @ old[:3, 2] > np.cos(np.deg2rad(4)) for old in self.mobility.attempted):
                continue
            visible = self._visible(proposals[:, 6:9], pose, tree, cancelled=cancelled)
            toward = proposals[:, 6:9]-pose[:3, 3]
            toward /= np.maximum(np.linalg.norm(toward, axis=1)[:, None], 1e-9)
            visible &= np.sum(toward*proposals[:, 3:6], axis=1) > .15
            count = int(visible.sum())
            if not count:
                continue
            arm_time = float(np.max(abs(q-current_q)/JointApproach.rates))
            base_time = max(np.linalg.norm(base_pose[:2]-current[:2])/p.s.base_speed_mps,
                            abs(angle(base_pose[2]-current[2]))/p.s.yaw_speed_radps)
            back_time = float(np.max(abs(q-self.return_q)/JointApproach.rates))
            score = count/(1.+max(arm_time, base_time)+back_time+p.s.settle_s)
            ranked.append((score, base_pose, q, pose, count, back_time))
        # Expensive swept geometry is bounded and score ordered; candidates
        # below the best feasible upper bound cannot improve that choice.
        certified_at = time.monotonic()
        for upper_score, base_pose, q, pose, count, back_time in sorted(ranked, key=lambda row: -row[0])[:16]:
            if cancelled():
                raise InterruptedError('mobile SEE certification cancelled')
            if time.monotonic()-certified_at > 5.:
                self.last_search['certificate_budget_exhausted'] = True
                break
            if best is not None and upper_score <= best['score']:
                break
            try:
                motion = self.motion(base_pose, q, snapshot, cancelled=cancelled)
            except FloorObservationRequired as request:
                self.last_search['unknown_floor'] += 1
                if missing is None:
                    missing = request
                continue
            if motion is None:
                self.last_search['rejected_paths'] += 1
                continue
            score = count/(1.+motion.travel_s+back_time+p.s.settle_s)
            if best is None or score > best['score']:
                best = dict(q=q.copy(), base_pose=base_pose.copy(), pose=pose,
                            motion=motion, path_certificate=motion.certificate, score=score,
                            expected_visibility_gain=count/len(proposals), visible_frontiers=count,
                            travel_s=motion.travel_s, return_s=back_time, policy='see')
            time.sleep(.001)
        self.last_search['reason'] = 'selected' if best else 'needs_floor' if missing is not None else 'no_mobile_view'
        if best is not None:
            self.last_search['selected_base_pose'] = best['base_pose'].tolist()
        return best, missing
