import threading
import time
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from elesim_pilot.reconstruction import PipelineCancelled
from elesim_pilot.reconstruction.live_scan import LiveScan
from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
from elesim_pilot.vision.perception.realsense_camera import RealSenseFrame
from elesim_protocol import SimMappingConfig, SimQ


@pytest.fixture
def rig(tmp_path):
    now, sent, released, frames = [10.], [], [], []
    host = NS(connected=True, q=SimQ(0., 0., 0., 0.), safety_fault='', reply_ok=True,
        reply_reason='', torque_enabled=True, claw_closed=False, go2_vel=(0., 0., 0.),
        go2_base_lin_vel_body=None, go2_base_ang_vel=None)
    state = NS(ik_running=False, set_q=lambda *q: None)
    c = NS(pilot_id='pilot', active_target='sim', desired_target='sim', lease_id='lease',
        mapping=SimMappingConfig(), dds_settings=None, state_sink=NS(get_state=lambda: host),
        endpoints=[{'endpoint_id': 'sim', 'instance_id': 'boot', 'role': 'sim', 'streams': {
            'rgbd': {'format': 'encoded-rgbd-v1', 'endpoint': '/sim/rgbd'}}}],
        clear_pending_motion=lambda: None,
        release_trajectory_route=lambda route: released.append(route))
    c.motion_targets = c.endpoints
    def send(q, route, **kw):
        sent.append((list(q), route, threading.get_ident(), kw))
        host.q = SimQ(*q)
    c.send_trajectory_q = send
    s = NS(_visual_busy=lambda: False, _pick_busy=lambda: False, _gaze_busy=lambda: False,
        pick_e2e_running=lambda: False, wrap_grasp_running=lambda: False,
        sim_reset_in_progress=False, _ik_context_for_host=lambda host: {'base_world_T': np.eye(4)})
    sim = NS(epoch=1, paused=False)
    class Camera:
        def __init__(self, **kw):
            self.kw, self.sequence = kw, 0
        def start(self): pass
        def stop(self): pass
        def capture(self, **kw):
            self.sequence += 1
            q = list(vars(host.q).values())
            frame = RealSenseFrame(np.zeros((10, 10, 3), np.uint8), np.ones((10,10), np.float32),
                1., CameraIntrinsics(20.,20.,5.,5.,10,10), seq=self.sequence, ts=1000.+now[0],
                source_id='sim', source_boot_id='boot', calibration_id='calibration', arm_q=q)
            frames.append(frame)
            return frame
    class Engine:
        def run(self, acquire, start, task, output, **kw):
            for step in range(2):
                sample = acquire(start+[-.001*step, 0., 0., 0.], {'accepted': True})
                assert sample.sensor.source_id == 'sim'
                np.testing.assert_allclose(sample.base_world_transform, np.eye(4))
            return None, {'stop_reason': 'view_budget', 'accepted_views': 2, 'geometry_complete_claimed': False}
    config = tmp_path/'models.yaml'
    config.write_text('test fixture')
    scan = LiveScan(c, s, state, simulation_status=lambda: sim, models_path=config,
        model_dir=tmp_path, hand_eye=tmp_path/'hand_eye.json', output_root=tmp_path/'scans',
        clock=lambda: now[0], camera_factory=Camera, engine_factory=lambda *a: Engine())
    def feedback(advance=True):
        now[0] += .05 if advance else 0.
        scan.accept_telemetry({'q_source': 'simulated', 'q': list(vars(host.q).values()),
            'measurement_time_s': 1000.+now[0], 'measurement_monotonic_s': now[0]},
            source_id='sim', source_boot_id='boot')
    def pump(predicate=lambda: not scan.active, timeout=4.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            feedback()
            scan.tick()
            time.sleep(.002)
        assert predicate(), scan.status()
    feedback()
    yield NS(scan=scan, c=c, service=s, state=state, host=host, sim=sim, now=now, sent=sent,
        released=released, frames=frames, feedback=feedback, pump=pump, camera=Camera, engine=Engine)
    scan.close()


def test_worker_acquires_after_settling_and_only_tick_sends_motion(rig):
    tick_thread = threading.get_ident()
    rig.scan.start(max_views=2)
    rig.pump()
    rig.scan.tick()
    status = rig.scan.status()
    assert status['phase'] == 'completed' and status['accepted_views'] == 2
    assert len(rig.frames) == 2 and rig.sent
    assert {row[2] for row in rig.sent} == {tick_thread}
    assert all(row[3]['source'] == 'object_scan' for row in rig.sent)
    assert rig.released == [('sim', 'boot', 'lease')]
    assert not status['geometry_complete_claimed']


def test_robot_target_never_starts_or_sends_scan_motion(rig):
    rig.c.endpoints[0]['role'] = 'robot'
    with pytest.raises(RuntimeError, match='Robot is unsupported'):
        rig.scan.start()
    assert not rig.sent


@pytest.mark.parametrize('change, message', [
    (lambda r: setattr(r.host, 'torque_enabled', False), 'torque ON'),
    (lambda r: setattr(r.host, 'claw_closed', True), 'open claws'),
    (lambda r: setattr(r.sim, 'paused', True), 'resume'),
    (lambda r: r.now.__setitem__(0, r.now[0]+1.), 'fresh Sim joint'),
])
def test_preflight_failure_never_moves(rig, change, message):
    change(rig)
    with pytest.raises(RuntimeError, match=message):
        rig.scan.start()
    assert not rig.sent


@pytest.mark.parametrize('change, message', [
    (lambda r: setattr(r.c, 'lease_id', 'new-lease'), 'boot/lease'),
    (lambda r: r.c.endpoints[0].update(instance_id='new-boot'), 'fresh Sim joint'),
    (lambda r: setattr(r.sim, 'epoch', 2), 'epoch'),
    (lambda r: setattr(r.sim, 'paused', True), 'resume'),
])
def test_target_session_change_stops_before_another_command(rig, change, message):
    rig.scan.start()
    count = len(rig.sent)
    change(rig)
    rig.scan.tick()
    assert rig.scan.status()['phase'] == 'failed'
    assert message in rig.scan.status()['reason']
    assert len(rig.sent) == count
    assert all(row[1][0] == 'sim' for row in rig.sent)


def test_cancel_releases_original_route_and_prevents_late_worker_completion(rig):
    rig.scan.start()
    rig.scan.cancel()
    rig.scan._worker.join(timeout=1.)
    assert rig.scan.status()['phase'] == 'cancelled'
    assert rig.released == [('sim', 'boot', 'lease')]
    before = len(rig.sent)
    rig.feedback()
    rig.scan.tick()
    assert len(rig.sent) == before


def test_repeated_feedback_does_not_count_as_three_settled_samples(rig):
    rig.scan.start()
    deadline = time.monotonic()+1.
    while rig.scan._motion is None and time.monotonic() < deadline:
        time.sleep(.002)
    assert rig.scan._motion is not None
    for _ in range(5):
        rig.now[0] += .05
        rig.scan.tick()  # No new source-monotonic sample.
    assert rig.scan._motion['stable'] == 1
    assert not rig.frames


def test_manual_and_competing_workflows_are_blocked_until_cancelled(rig):
    rig.scan.start()
    assert not rig.scan.allow_motion({'command': 'target'})
    with pytest.raises(RuntimeError, match='cancel it'):
        rig.scan.guard_operator('service_call', 'start_wrap_grasp')
    rig.scan.guard_operator('service_call', 'reset_simulation')
    assert rig.scan.status()['phase'] == 'cancelled'


def test_control_stall_fails_without_skipping_or_bursting_waypoints(rig):
    rig.scan.start()
    rig.now[0] += .7
    rig.feedback()
    rig.scan.tick()
    assert rig.scan.status()['phase'] == 'failed'
    assert 'tick stalled' in rig.scan.status()['reason']
    assert not rig.sent


def test_async_cancellation_between_preflight_and_send_blocks_that_tick(rig):
    rig.scan.start()
    rig.pump(lambda: rig.scan._motion is not None and rig.scan._motion['goal'][0] < 0.)
    assert rig.scan._last_q[0] == 0.
    checked_feedback = rig.scan._checked_feedback
    def cancel_after_feedback():
        values = checked_feedback()
        rig.scan.request_cancel('simulation paused during tick')
        return values
    rig.scan._checked_feedback = cancel_after_feedback
    rig.feedback()
    before = len(rig.sent)
    rig.scan.tick()
    assert len(rig.sent) == before
    assert rig.scan.status()['phase'] == 'cancelled'
    assert rig.released == [('sim', 'boot', 'lease')]


def test_camera_calibration_change_fails_the_session(rig):
    original = rig.camera.capture
    def changed(camera, **kw):
        frame = original(camera, **kw)
        if camera.sequence > 1:
            frame.calibration_id = 'changed'
        return frame
    rig.camera.capture = changed
    rig.scan.start()
    rig.pump()
    assert rig.scan.status()['phase'] == 'failed'
    assert 'calibration changed' in rig.scan.status()['reason']


def test_stale_camera_frame_is_skipped_until_post_settle_capture(rig):
    original = rig.camera.capture
    def stale(camera, **kw):
        frame = original(camera, **kw)
        if camera.sequence == 1:
            frame.ts = 1.
        return frame
    rig.camera.capture = stale
    rig.scan.start()
    rig.pump()
    assert rig.scan.status()['phase'] == 'completed'
    assert len(rig.frames) == 3


def test_scan_never_sends_a_view_outside_an_operating_cap(rig):
    rig.c.mapping = replace(rig.c.mapping, linear_u_limit=0.)
    rig.scan.start()
    rig.pump()
    assert rig.scan.status()['phase'] == 'failed'
    assert 'operating bounds' in rig.scan.status()['reason']
    assert all(q[0] == 0. for q, *_ in rig.sent)


def test_completed_status_merges_progress_with_final_report(rig):
    class Engine:
        def run(self,*args,**kwargs):
            result,report = rig.engine().run(*args,**kwargs)
            kwargs['progress'](dict(stage='reconstructing',accepted_views=2,observed_points=25,
                coverage={'fraction':.1},stop_reason='view_budget'))
            report.update(observed_points=30,coverage={'fraction':.25})
            return result,report
    rig.scan.engine_factory = lambda *_args:Engine()
    rig.scan.start(max_views=2)
    rig.pump()
    rig.scan.tick()
    status = rig.scan.status()
    assert status['phase'] == 'completed' and not status['active']
    assert status['accepted_views'] == 2 and status['observed_points'] == 30
    assert status['coverage'] == {'fraction':.25}
    assert status['stop_reason'] == 'view_budget'
