from types import SimpleNamespace as NS

import pytest

from workbench.research.debug.profile_sim_walking import wait_for_camera_frames


def fixture():
    state = NS(time=0., versions={'hand_eye_preview': 0, 'observer': 0}, captures=[])
    cfg = NS(camera_first_frame_timeout_s=.1, sim_camera_max_hz=30.,
        sim_camera_rgb=True, sim_camera_depth=True, sim_observer_camera_max_hz=30.)
    scene = NS(camera_render_worker=NS(failure=''),
        maybe_publish_camera=lambda **kwargs: state.captures.append(('hand_eye_preview', kwargs)),
        maybe_publish_observer_camera=lambda **kwargs: state.captures.append(('observer', kwargs)))
    hub = NS(version=lambda name: state.versions[name])
    return state, cfg, scene, hub


def test_short_trial_waits_for_both_first_frames_without_advancing_physics():
    state, cfg, scene, hub = fixture()
    def sleep(seconds):
        state.time += seconds
        state.versions['hand_eye_preview'] = int(state.time >= .02)
        state.versions['observer'] = int(state.time >= .04)
    elapsed = wait_for_camera_frames(scene, hub, cfg, clock=lambda: state.time, sleep=sleep)
    assert elapsed == pytest.approx(.04)
    assert all(kwargs['sim_time_s'] == 0. for _, kwargs in state.captures)
    assert {name for name, _ in state.captures} == {'hand_eye_preview', 'observer'}


def test_failed_renderer_aborts_before_starting_a_trial():
    state, cfg, scene, hub = fixture()
    scene.camera_render_worker.failure = 'EGL unavailable'
    with pytest.raises(RuntimeError, match='EGL unavailable'):
        wait_for_camera_frames(scene, hub, cfg, clock=lambda: state.time)
    assert not state.captures


def test_first_frame_timeout_uses_wall_time_even_when_simulation_is_stationary():
    state, cfg, scene, hub = fixture()
    def sleep(seconds):
        state.time += seconds
    with pytest.raises(TimeoutError, match='both first frames'):
        wait_for_camera_frames(scene, hub, cfg, clock=lambda: state.time, sleep=sleep)
    assert .1 <= state.time < .12
