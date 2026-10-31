"""Sim-only scan lifecycle. DDS tick owns all motion, worker owns all models/I/O."""
from __future__ import annotations

import threading
import time
import uuid
from pathlib import Path

import numpy as np
from elesim_protocol import linear_effective_q_bounds

from elesim_pilot.arm_trajectory import require_stable_arm_base
from elesim_pilot.robot.arm.iklib.kinematics import base_world_transform_from_context
from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
from elesim_pilot.vision.multiview import ViewPlanner
from elesim_pilot.vision.partial_cloud import ArmCameraModel, SensorObservation
from elesim_pilot.vision.perception.sim_rendered_camera import SimRenderedCamera
from .contracts import Measurement, Task
from .pipeline import Pipeline, PipelineCancelled
from .scan import ActiveScan
from .scan_collision import ScanCollision
from .artifacts import scene_publication


class LiveScan:
    def __init__(self, connection, service, state, *, simulation_status, models_path,
                 model_dir, hand_eye, output_root, other_busy=lambda: False,
                 clock=time.monotonic, camera_factory=SimRenderedCamera, engine_factory=None):
        self.connection, self.service, self.state = connection, service, state
        self.simulation_status, self.other_busy = simulation_status, other_busy
        self.models_path, self.model_dir = Path(models_path), Path(model_dir)
        self.hand_eye, self.output_root = Path(hand_eye), Path(output_root)
        self.clock, self.camera_factory, self.engine_factory = clock, camera_factory, engine_factory
        self._condition = threading.Condition(threading.RLock())
        self._stop = threading.Event()
        self._worker = None
        self._feedback = None
        self._generation = 0
        self._phase, self._reason, self._job_id = 'idle', '', ''
        self._progress, self._result = {}, {}
        self._motion = None
        self._route = None
        self._release_pending = False
        self._cancel_reason = ''

    @property
    def active(self):
        return self._phase not in {'idle', 'completed', 'cancelled', 'failed'}

    def route(self):
        c = self.connection
        descriptor = next((d for d in c.endpoints if d.get('endpoint_id') == c.active_target), {})
        if (descriptor.get('role') != 'sim' or not descriptor.get('instance_id') or not c.lease_id
                or c.active_target != c.desired_target
                or c.active_target not in {d['endpoint_id'] for d in c.motion_targets}):
            raise RuntimeError('object scan requires a selectable Sim boot and motion lease; Robot is unsupported')
        return c.active_target, str(descriptor['instance_id']), c.lease_id

    def accept_telemetry(self, payload, *, source_id, source_boot_id):
        if payload.get('q_source') != 'simulated' or 'q' not in payload:
            return
        q = np.asarray(payload['q'], dtype=float)
        wall, mono = payload.get('measurement_time_s'), payload.get('measurement_monotonic_s')
        if (q.shape != (4,) or not np.isfinite(q).all() or
                any(isinstance(t, bool) or not isinstance(t, (int, float)) or not np.isfinite(t) or t <= 0
                    for t in (wall, mono))):
            return
        with self._condition:
            old = self._feedback
            identity = (str(source_id), str(source_boot_id))
            if old is not None and old['identity'] == identity and mono <= old['mono']:
                return
            self._feedback = dict(identity=identity, q=q.copy(), wall=float(wall), mono=float(mono),
                                  received=self.clock())

    def _checked_feedback(self):
        route = self.route()
        with self._condition:
            feedback = self._feedback
        if (feedback is None or feedback['identity'] != route[:2]
                or self.clock()-feedback['received'] > .5):
            raise RuntimeError('fresh Sim joint measurements with source timestamps required (<=500 ms)')
        host = self.connection.state_sink.get_state()
        if not host.connected or host.safety_fault or not host.reply_ok:
            raise RuntimeError(host.safety_fault or host.reply_reason or 'Sim disconnected/rejected command')
        if not host.torque_enabled or host.claw_closed is not False:
            raise RuntimeError('turn Sim torque ON and open claws before scanning')
        if (max(abs(v) for v in host.go2_vel) > 1e-4
                or (host.go2_base_lin_vel_body is not None and np.linalg.norm(host.go2_base_lin_vel_body) > .01)
                or (host.go2_base_ang_vel is not None and np.linalg.norm(host.go2_base_ang_vel) > .05)):
            raise RuntimeError('scan requires a stationary arm base')
        if getattr(self.service, '_go2_arm_mount', None) is not None:
            if (host.go2_base_pos is None or host.go2_base_rpy is None
                    or abs(feedback['wall']-host.go2_base_timestamp_s) > .2):
                raise RuntimeError('fresh measured Sim base pose required')
        context = self.service._ik_context_for_host(host)
        base = base_world_transform_from_context(context)
        status = self.simulation_status()
        if status is None or status.paused:
            raise RuntimeError('running Sim session status required; resume before scanning')
        return feedback, base, status

    def _busy(self):
        s = self.service
        return (self.other_busy() or s._visual_busy() or s._pick_busy() or s._gaze_busy()
                or s.pick_e2e_running() or s.wrap_grasp_running() or self.state.ik_running
                or s.sim_reset_in_progress)

    def status(self):
        blocked = ''
        if not self.active:
            try:
                self._checked_feedback()
                if self._busy():
                    raise RuntimeError('stop other arm workflows before scanning')
                if self._worker is not None and self._worker.is_alive():
                    raise RuntimeError('previous scan worker is still stopping')
                if self._release_pending:
                    raise RuntimeError('previous scan lease release is pending')
                if not self.models_path.is_file():
                    raise RuntimeError('scan model configuration unavailable on Pilot')
            except (RuntimeError, ValueError) as exc:
                blocked = str(exc)
        with self._condition:
            return dict(phase=self._phase, active=self.active, job_id=self._job_id,
                reason=self._reason, blocked_reason=blocked, can_start=not self.active and not blocked,
                geometry_complete_claimed=False, **{**self._progress, **self._result})

    def start(self, target_id='target', max_views=12):
        if self.active or (self._worker is not None and self._worker.is_alive()) or self._release_pending:
            raise RuntimeError('scan session already active or stopping')
        if self._busy():
            raise RuntimeError('stop other arm workflows before scanning')
        feedback, base, status = self._checked_feedback()
        mapping = self.connection.mapping
        bounds = np.array([linear_effective_q_bounds(mapping),
            [mapping.roll_q_min_rad, mapping.roll_q_max_rad],
            [mapping.seg1_q_min_rad, mapping.seg1_q_max_rad],
            [mapping.seg2_q_min_rad, mapping.seg2_q_max_rad]])
        if not np.isfinite(bounds).all() or np.any(feedback['q'] < bounds[:, 0]) or np.any(feedback['q'] > bounds[:, 1]):
            raise ValueError('measured scan start is outside configured operating bounds')
        if type(max_views) is not int or not 2 <= max_views <= 24:
            raise ValueError('scan view budget must be 2..24')
        job_id = uuid.uuid4().hex
        task = Task('scan-'+job_id, target_id)
        route = self.route()
        descriptor = next(d for d in self.connection.endpoints if d['endpoint_id'] == route[0])
        stream = descriptor.get('streams', {}).get('rgbd', {})
        if stream.get('format') != 'encoded-rgbd-v1' or not stream.get('endpoint'):
            raise RuntimeError('scan requires encoded RGB-D with source identity, calibration and measured q')
        self._generation += 1
        generation = self._generation
        self._stop.clear()
        self._cancel_reason = ''
        self._job_id, self._route, self._base, self._epoch = job_id, route, base.copy(), status.epoch
        self._phase, self._reason, self._progress, self._result = 'preparing', '', {}, {}
        self._last_q, self._last_send, self._started = feedback['q'].copy(), self.clock(), self.clock()
        self._motion = None
        self._bounds = bounds
        self.connection.clear_pending_motion()

        def cancelled():
            return self._stop.is_set() or generation != self._generation

        def progress(event):
            with self._condition:
                if not cancelled():
                    self._phase = str(event.get('stage', self._phase))
                    self._progress.update({k: event[k] for k in ('stage', 'view', 'views', 'accepted_views',
                        'observed_points', 'coverage', 'stop_reason') if k in event})

        def worker():
            camera = None
            pipeline = None
            try:
                progress({'stage': 'loading_models'})
                if self.engine_factory is None:
                    pipeline = Pipeline.from_config(self.models_path, mapping=self.connection.mapping)
                    pipeline.prepare(cancelled=cancelled, progress=progress)
                    model = ArmCameraModel(self.model_dir.parents[1]/'arm/default.json', self.hand_eye,
                        base_world_transform=base)
                    arm = ArmSelfCollision(self.model_dir, joint_uncertainty=[.003, .04, .04, .04])
                    collision = ScanCollision(arm, base)
                    planner = ViewPlanner(model, collision_checker=collision, cancelled=cancelled,
                        operating_bounds=bounds)
                    engine = ActiveScan(pipeline, model, planner, max_views=max_views)
                else:
                    engine = self.engine_factory(max_views, cancelled)
                if cancelled():
                    raise PipelineCancelled('scan cancelled')
                camera = self.camera_factory(topic=stream['endpoint'], timeout_ms=100,
                    endpoint_id=self.connection.pilot_id+'-scan', dds_settings=self.connection.dds_settings,
                    expected_source_id=route[0], expected_boot_id=route[1], wire_format=stream['format'])
                camera.start()
                last_seq, calibration = -1, None
                settled_goal, settled_wall = None, None

                def acquire(goal, certificate):
                    nonlocal last_seq, calibration, settled_goal, settled_wall
                    if not certificate.get('accepted'):
                        raise ValueError('uncertified scan motion')
                    feedback, _, _ = self._checked_feedback()
                    if (settled_goal is None or not np.array_equal(goal, settled_goal)
                            or not np.allclose(feedback['q'], goal, rtol=0., atol=[.001, .015, .015, .015])):
                        settled_wall = self._move_and_wait(goal, certificate, generation, cancelled)
                        settled_goal = np.array(goal, copy=True)
                    progress({'stage': 'capturing'})
                    deadline = self.clock()+8.
                    while not cancelled() and self.clock() < deadline:
                        try:
                            frame = camera.capture(retries=1, retry_sleep_s=0.)
                        except RuntimeError:
                            continue
                        if (frame.source_id, frame.source_boot_id) != route[:2]:
                            raise ValueError('scan RGB-D source boot changed')
                        if calibration is not None and frame.calibration_id != calibration:
                            raise ValueError('camera calibration changed during scan')
                        current_feedback, _, _ = self._checked_feedback()
                        if (frame.seq <= last_seq or frame.ts < settled_wall
                                or not -.2 <= current_feedback['wall']-frame.ts <= .5 or frame.arm_q is None
                                or not np.allclose(frame.arm_q, goal, rtol=0., atol=[.001, .015, .015, .015])):
                            time.sleep(.001)
                            continue
                        sample = SensorObservation(frame.color_bgr, frame.depth_raw, frame.depth_scale,
                            frame.intrinsics, frame.seq, frame.ts, frame.source_id, frame.source_boot_id,
                            frame.calibration_id, tuple(frame.arm_q))
                        calibration, last_seq = frame.calibration_id, frame.seq
                        return Measurement(sample, base_world_transform=base).validated()
                    if cancelled():
                        raise PipelineCancelled('scan cancelled')
                    raise TimeoutError('no fresh coherent RGB-D after measured scan settling')

                _, report = engine.run(acquire, feedback['q'], task, self.output_root/job_id,
                    cancelled=cancelled, progress=progress)
                publication = (scene_publication(report['scene_manifest'], self.output_root, *route[:2])
                    if report.get('scene_manifest') else {})
                with self._condition:
                    if not cancelled():
                        self._phase = 'completed'
                        self._progress['stage'] = 'completed'
                        self._result = {key: report[key] for key in ('scene_manifest', 'stop_reason',
                            'accepted_views', 'observed_points', 'predicted_points', 'model_execution',
                            'coverage', 'needs_more_observation') if key in report}
                        self._result.update(publication)
                        self._release_pending = True
            except Exception as exc:
                with self._condition:
                    if not cancelled():
                        self._phase, self._reason = 'failed', str(exc)[:512]
                        self._release_pending = True
            finally:
                try:
                    if camera is not None:
                        camera.stop()
                finally:
                    if pipeline is not None:
                        pipeline.close()
                with self._condition:
                    self._condition.notify_all()

        self._worker = threading.Thread(target=worker, name='pilot-object-scan', daemon=True)
        self._worker.start()
        return self.status()

    def _move_and_wait(self, goal, certificate, generation, cancelled):
        if not certificate.get('accepted'):
            raise ValueError('uncertified scan motion')
        goal = np.asarray(goal, dtype=float)
        if (goal.shape != (4,) or not np.isfinite(goal).all() or np.any(goal < self._bounds[:, 0])
                or np.any(goal > self._bounds[:, 1])):
            raise ValueError('scan view exceeds configured operating bounds')
        with self._condition:
            if cancelled():
                raise PipelineCancelled('scan cancelled')
            now = self.clock()
            duration = float(np.max(np.abs(goal-self._last_q)/[.02, .15, .15, .15]))
            motion = dict(start=self._last_q.copy(), goal=np.array(goal), started=now,
                duration=max(.1, duration), deadline=now+max(12., duration*4.+5.),
                stable=0, last_sample=None, settled_wall=None)
            self._motion = motion
            while motion['settled_wall'] is None and not cancelled():
                if self.clock() > motion['deadline']:
                    raise TimeoutError('scan motion/settling deadline exceeded')
                self._condition.wait(.1)
            if cancelled() or generation != self._generation:
                raise PipelineCancelled('scan cancelled')
            return motion['settled_wall']

    def request_cancel(self, reason='operator cancelled'):
        with self._condition:
            if self.active:
                self._cancel_reason = str(reason)
                self._stop.set()
                self._condition.notify_all()

    def cancel(self):
        self.request_cancel()
        self.tick()
        return self.status()

    def _stop_on_tick(self, phase, reason):
        with self._condition:
            self._generation += 1
            self._stop.set()
            self._phase, self._reason = phase, str(reason)[:512]
            self._motion = None
            self._release_pending = True
            self._condition.notify_all()

    def tick(self):
        if self._stop.is_set() and self.active:
            self._stop_on_tick('cancelled', self._cancel_reason or 'scan cancelled')
        if self.active:
            try:
                now = self.clock()
                feedback, base, status = self._checked_feedback()
                if self.route() != self._route or status.epoch != self._epoch:
                    raise RuntimeError('scan target boot/lease or simulation epoch changed')
                require_stable_arm_base(self._base, base)
                if now-self._started > 600.:
                    raise TimeoutError('scan session exceeded 600 seconds')
                if now-self._last_send > .5:
                    raise TimeoutError('scan control tick stalled; stopped without catching up')
                if np.any(np.abs(feedback['q']-self._last_q) > [.003, .04, .04, .04]):
                    raise RuntimeError('Sim did not track the scan command within modeled uncertainty')
                if now-self._last_send >= .04:
                    with self._condition:
                        if self._stop.is_set():
                            raise PipelineCancelled(self._cancel_reason or 'scan cancelled')
                        motion = self._motion
                        q = self._last_q
                        if motion is not None:
                            if now > motion['deadline']:
                                raise TimeoutError('scan settling deadline exceeded')
                            fraction = min(1., max(0., (now-motion['started'])/motion['duration']))
                            q = motion['start']+(motion['goal']-motion['start'])*fraction
                            if fraction == 1. and motion['settled_wall'] is None:
                                close = np.all(np.abs(feedback['q']-motion['goal']) <= [.001, .015, .015, .015])
                                if feedback['mono'] != motion['last_sample']:
                                    motion['stable'] = motion['stable']+1 if close else 0
                                    motion['last_sample'] = feedback['mono']
                                if motion['stable'] >= 3:
                                    motion['settled_wall'] = feedback['wall']
                                    self._condition.notify_all()
                        if self.active:
                            self.connection.send_trajectory_q(list(map(float, q)), self._route, source='object_scan')
                            self._last_q, self._last_send = q.copy(), now
            except PipelineCancelled as exc:
                self._stop_on_tick('cancelled', str(exc))
            except Exception as exc:
                self._stop_on_tick('failed', str(exc))
        if self._release_pending:
            self._release_pending = False
            try:
                self.connection.clear_pending_motion()
                self.connection.release_trajectory_route(self._route)
            except Exception as exc:
                self._reason = (self._reason+'; lease release failed: '+str(exc))[:512]
            feedback = self._feedback
            if feedback is not None and self._route is not None and feedback['identity'] == self._route[:2]:
                try:
                    if self.route() == self._route:
                        self.state.set_q(*map(float, feedback['q']))
                except RuntimeError:
                    pass

    def guard_operator(self, operation, name):
        if not self.active or operation == 'view_snapshot':
            return
        if operation == 'service_call' and name in {'cancel_object_scan', 'torque_off', 'reset_simulation',
                                                   'disconnect_device', 'select_endpoint'}:
            self._stop_on_tick('cancelled', name)
            self.tick()
            return
        raise RuntimeError('object scan owns arm motion; cancel it before other operator changes')

    def allow_motion(self, payload):
        if not self.active:
            return True
        if payload.get('command') in {'estop', 'torque_off'}:
            self._stop_on_tick('cancelled', payload['command'])
            return True
        return False

    def close(self, *, timeout_s=5.):
        self.request_cancel('Pilot stopping')
        worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=max(0., float(timeout_s)))
            if worker.is_alive():
                raise TimeoutError('object scan worker has not stopped')
