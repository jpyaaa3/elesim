"""Mobile scan acquisition: a worker requests views; DDS ticks execute motion."""
import threading

import numpy as np
from scipy.spatial.transform import Rotation

from elesim_pilot.arm_trajectory import require_stable_arm_base, arm_base_pose_delta
from elesim_pilot.reconstruction.contracts import Measurement
from elesim_pilot.reconstruction.pipeline import ObservationPending
from elesim_pilot.robot.arm.approach import JointApproach
from .base_executor import BaseExecutor
from .contracts import BaseGoal, planar
from .see_planner import MobileSeePlanner, ViewMotion


class ScanMotion:
    def __init__(self, ports, operation):
        self.p, self.op = ports, operation
        self.condition = threading.Condition()
        self.request = None
        self.last_q = ports.feedback.snapshot(ports.route()[:2])['joint']['q'].copy()
        self.last_send = ports.clock()
        self.last_settled = None
        self.attempted = []
        self.error = None
        self.bounds = ports.arm.bounds.copy()

    def make_planner(self, model, **options):
        self.planner = MobileSeePlanner(model, self.p, self, **options)
        self.bounds = self.planner.operating_bounds.copy()
        self.p.camera_q = self.planner.q
        self.p.camera_cache = self.planner.local_poses
        return self.planner

    def _wait(self, plan, cancelled):
        p = self.p
        goal = np.asarray(plan.base.arm_q, float)
        if np.any(goal < self.bounds[:, 0]) or np.any(goal > self.bounds[:, 1]):
            raise ValueError('mobile scan view exceeds configured operating bounds')
        snapshot = p.feedback.snapshot(p.route()[:2])
        actual_base = p.service._go2_arm_mount.arm_base_world_transform(
            snapshot['base']['pos'], snapshot['base']['rpy'])
        require_stable_arm_base(plan.start_arm_base, actual_base)
        if np.any(abs(snapshot['joint']['q']-plan.start_q)>JointApproach.tolerance):
            raise RuntimeError('scan motion start changed during planning')
        executor = BaseExecutor(p.s, p.clock)
        executor.start(plan.base, snapshot['pose'], p.grid.revision)
        request = dict(plan=plan, executor=executor,
                       approach=JointApproach(plan.start_q, plan.base.arm_q),
                       last_sim=snapshot['joint']['sim_time'], started=p.clock(),
                       stable=None, sample=None, samples=0, result=None)
        with self.condition:
            if self.request is not None:
                raise RuntimeError('mobile scan motion already pending')
            self.request = request
            while request['result'] is None:
                if cancelled() or self.error is not None:
                    raise InterruptedError(self.error or 'mobile scan cancelled')
                if p.clock()-request['started']>180.:
                    raise TimeoutError('mobile scan view exceeded 180 seconds')
                self.condition.wait(.1)
            return request['result']

    def relocate(self, choice, *, cancelled):
        # Rebuild the selected certificate using fresh measured joints and the
        # current map before authorizing either actuator group.
        snapshot = self.p.feedback.snapshot(self.p.route()[:2])
        self.planner.update_base(snapshot)
        plan = self.planner.motion(np.asarray(choice['base_pose']), np.asarray(choice['q']),
                                   snapshot, cancelled=cancelled)
        if plan is None:
            raise RuntimeError('selected mobile view lost motion admission')
        result = self._wait(plan, cancelled)
        self.planner.update_base(self.p.feedback.snapshot(self.p.route()[:2]))
        self.attempted.append(self.planner.model.camera_pose(result['q']))
        # History in another base frame cannot be used for an arm-only return.
        self.p.history = [result['q'].copy()]
        return result['q']

    def settle(self, goal, *, cancelled):
        snapshot = self.p.feedback.snapshot(self.p.route()[:2])
        if (self.last_settled is not None and
                np.all(abs(snapshot['joint']['q']-goal)<=JointApproach.tolerance)):
            base = self.p.service._go2_arm_mount.arm_base_world_transform(
                snapshot['base']['pos'], snapshot['base']['rpy'])
            require_stable_arm_base(self.last_settled['base'], base)
            return self.last_settled['wall']
        self.planner.update_base(snapshot)
        certificate = self.p.collision.certify_segment(snapshot['joint']['q'], goal)
        if not certificate['accepted']:
            raise RuntimeError('initial scan posture is not admitted')
        # This initial measurement holds the already reached pose. The extra
        # arrival reserve applies when choosing a new view, while this hold
        # retains the ordinary uncertainty-aware segment certificate.
        base_goal = BaseGoal(snapshot['pose'], snapshot['pose'][None], goal, dict(kind='observation'))
        plan = ViewMotion(base_goal, snapshot['joint']['q'], self.p.footprint, certificate,
                          self.p.collision.base.copy(), 0.)
        return self._wait(plan, cancelled)['wall']

    def measurement(self, frame):
        from elesim_pilot.vision.partial_cloud import SensorObservation
        p = self.p
        try:
            body = p.feedback.associate(frame, p.route()[:2])
        except RuntimeError as exc:
            raise ObservationPending('capture-time base feedback unavailable') from exc
        base = p.service._go2_arm_mount.arm_base_world_transform(body['pos'], body['rpy'])
        require_stable_arm_base(self.last_settled['base'], base)
        sensor = SensorObservation(frame.color_bgr, frame.depth_raw, frame.depth_scale,
            frame.intrinsics, frame.seq, frame.ts, frame.source_id, frame.source_boot_id,
            frame.calibration_id, tuple(frame.arm_q))
        return Measurement(sensor, base_world_transform=base).validated()

    def observe_scene(self, measurement, pose):
        # Reuse the mobile robot-body exclusion. The standalone scan observer
        # removes arm spheres only, so sharing it here would insert Go2's own
        # visible body into the persistent obstacle map.
        # This transform was constructed from the source-matched body sample
        # at acquisition. Inference may outlive the feedback history window.
        measured_body = measurement.base_world_transform @ np.linalg.inv(self.p.arm_in_body)
        body = dict(pos=measured_body[:3, 3],
                    rpy=Rotation.from_matrix(measured_body[:3, :3]).as_euler('xyz'))
        self.p._map_scene(measurement, pose, measurement.base_world_transform, body)

    def observe_floor(self, request, *, cancelled):
        p = self.p
        index = p.reserve_floor_view()
        p.mapping_goal_pose = request.pose.copy()
        p.mapping_goal_xy = request.pose[:2].copy()
        p.mapping_goal_footprint = request.footprint
        p.mapping_margin_m = request.margin_m
        points = p._map_search_prepare(index)
        for q in points[1:]:
            snapshot = p.feedback.snapshot(p.route()[:2])
            self.planner.update_base(snapshot)
            certificate = p.collision.certify_segment(snapshot['joint']['q'], q)
            if not certificate['accepted']:
                raise RuntimeError('floor observation route lost admission')
            base_goal = BaseGoal(snapshot['pose'], snapshot['pose'][None], q, dict(kind='observation'))
            plan = ViewMotion(base_goal, snapshot['joint']['q'], p.footprint, certificate,
                              p.collision.base.copy(), float(np.max(abs(q-snapshot['joint']['q'])/JointApproach.rates)))
            self._wait(plan, cancelled)
        measurement, pose, base = p._capture()
        if cancelled():
            raise InterruptedError('mobile floor capture cancelled')
        p._map_scene(measurement, pose, base, p.feedback.associate(measurement.sensor, p.route()[:2]))
        p.history.extend(np.asarray(q).copy() for q in points[1:])
        p._summary['active_scan_floor_views'] = len(p.mapping_views)

    def tick(self):
        p = self.p
        now = p.clock()
        if self.error is not None:
            raise InterruptedError(self.error)
        if now-self.last_send>.5:
            raise TimeoutError('mobile scan control tick stalled')
        if now-self.last_send<.04:
            return
        snapshot = p.feedback.snapshot(p.route()[:2])
        joint, body = snapshot['joint'], snapshot['base']
        if np.any(abs(joint['q']-self.last_q)>JointApproach.tolerance):
            raise RuntimeError('mobile scan arm left its certified tracking tube')
        velocity = np.zeros(3)
        q = self.last_q.copy()
        with self.condition:
            request = self.request
            if request is not None:
                if joint['sim_time'] is None or joint['sim_time']<request['last_sim']:
                    raise RuntimeError('mobile scan Sim clock missing or reset')
                dt = min(.1, joint['sim_time']-request['last_sim'])
                request['last_sim'] = joint['sim_time']
                q = request['approach'].step(joint['q'], dt)
                plan = request['plan']
                moving = len(plan.base.path)>1
                if moving:
                    expected=(planar(snapshot['pose']) @ np.linalg.inv(planar(plan.base.path[0]))
                              @ plan.start_arm_base)
                    actual=p.service._go2_arm_mount.arm_base_world_transform(body['pos'],body['rpy'])
                    translation,rotation=arm_base_pose_delta(expected,actual)
                    # Account for the measured walking attitude geometrically;
                    # walking sway itself is not a task-failure threshold.
                    sway_margin=translation+2*plan.reach_m*np.sin(rotation/2)
                    if not p.grid.footprint_free(snapshot['pose'],plan.footprint,margin_m=sway_margin):
                        raise RuntimeError('mobile scan sweep left observed free floor')
                    velocity, base_done = request['executor'].step(snapshot['pose'],
                        np.linalg.norm(body['velocity']), np.linalg.norm(body['angular']),
                        lambda a,b: p.grid.segment_free(a,b,plan.footprint,margin_m=sway_margin),
                        posture_ready=(body['locomotion_stage']=='stand' and
                                       np.max(abs(body['rpy'][:2]))<=p.s.standing_tilt_tolerance_rad))
                else:
                    require_stable_arm_base(plan.start_arm_base,
                        p.service._go2_arm_mount.arm_base_world_transform(body['pos'],body['rpy']))
                    base_done = (np.linalg.norm(body['velocity'])<.01 and
                                 np.linalg.norm(body['angular'])<.05 and body['locomotion_stage']=='stand')
                close = np.all(abs(joint['q']-plan.base.arm_q)<=JointApproach.tolerance)
                if base_done and request['approach'].fraction==1. and close:
                    if request['stable'] is None:
                        request['stable'] = now
                    if request['sample']!=joint['mono']:
                        request['sample'] = joint['mono']
                        request['samples'] += 1
                    if now-request['stable']>=p.s.settle_s and request['samples']>=3:
                        base = p.service._go2_arm_mount.arm_base_world_transform(body['pos'],body['rpy'])
                        q = joint['q'].copy()
                        result = dict(q=q.copy(),wall=joint['stamp'],base=base.copy())
                        self.last_settled = result
                        request['result'] = result
                        self.request = None
                        self.condition.notify_all()
                else:
                    request['stable'] = None
                    request['samples'] = 0
                p._summary['active_view'] = dict(base_goal=plan.base.pose.tolist(),
                    arm_goal=plan.base.arm_q.tolist(),arm_fraction=request['approach'].fraction,
                    base_stage=request['executor'].stage if moving else 'stationary',
                    map_revision=p.grid.revision)
            p._send(self.op['grant'], self.op['ownership'], q=q.tolist(), go2_vel=velocity.tolist())
            self.last_q, self.last_send = q.copy(), now
            p.last_send = now

    def cancel(self, reason):
        with self.condition:
            self.error = str(reason)
            self.condition.notify_all()
        # Parent cancellation also stops its child. Route/grant checks remain
        # authoritative when a failure originated in LiveScan instead.
        if not self.p.stopped:
            self.p._send(self.op['grant'], self.op['ownership'], go2_vel=[0.,0.,0.])
