import multiprocessing as mp
from types import SimpleNamespace as NS
import threading

import pytest

from elesim_sim.vision.sim_camera.async_worker import (
    CameraRenderSpec, CameraRenderWorker, CameraStateSnapshot, _CameraSceneEdit,
    _add_base_entity,
)


def worker():
    spec = CameraRenderSpec(urdf_path='visual.urdf', robot_pos=(0., 0., 0.),
        robot_euler_deg=(0., 0., 0.), use_gpu=False, gpu_convert=False,
        dt=.01, gravity=(0., 0., 0.), substeps=1, floor=True)
    value = CameraRenderWorker(spec, {'observer': (2, 2), 'hand_eye_preview': (2, 2)},
        lambda *args: None, context=mp.get_context('spawn'))
    for state in value._processes.values():
        state.process = NS(is_alive=lambda: False)
    value._started = True
    return value


def test_scene_edit_is_reliable_and_waits_for_both_current_epoch_acknowledgements():
    value = worker()
    errors, seen = [], []

    def respond():
        try:
            for stream, state in value._processes.items():
                edit = state.commands.get(timeout=2.)
                assert isinstance(edit, _CameraSceneEdit)
                assert not value.submit(CameraStateSnapshot(1, 0, 0.), ('observer',))
                seen.append((stream, edit))
                value._receive_message({'type':'scene_rebuilt', 'epoch':0, 'ok':True}, stream_hint=stream)
                assert not value._edit_done.is_set()
                value._receive_message({'type':'scene_rebuilt', 'epoch':edit.epoch, 'ok':True}, stream_hint=stream)
        except BaseException as error:
            errors.append(error)

    thread = threading.Thread(target=respond)
    thread.start()
    try:
        value.rebuild_scene('new/scene.json', 'a'*64, epoch=1, timeout_s=3.)
        thread.join(2.)
        assert not thread.is_alive() and not errors
        assert len(seen) == 2 and value.spec.scene_revision == 'a'*64
        with pytest.raises(ValueError, match='advance'):
            value.rebuild_scene('', '', epoch=1, timeout_s=.1)
    finally:
        value.close()


def test_failed_camera_edit_does_not_commit_spec_and_worker_can_accept_rollback():
    value = worker()

    def respond():
        for epoch in (1, 2):
            for stream, state in value._processes.items():
                edit = state.commands.get(timeout=2.)
                assert edit.epoch == epoch
                value._receive_message({'type':'scene_rebuilt', 'epoch':epoch,
                    'ok':epoch == 2, 'error':'bad mesh'}, stream_hint=stream)

    thread = threading.Thread(target=respond)
    thread.start()
    try:
        with pytest.raises(RuntimeError, match='bad mesh'):
            value.rebuild_scene('bad/scene.json', 'a'*64, epoch=1, timeout_s=3.)
        assert value.spec.scene_manifest == '' and not value.failure
        value.rebuild_scene('', '', epoch=2, timeout_s=3.)
        thread.join(2.)
        assert not thread.is_alive()
    finally:
        value.close()


def test_preparation_runs_before_wait_and_does_not_commit_or_accept_snapshots_early():
    value = worker()
    try:
        ticket = value.begin_scene_rebuild('new/scene.json', 'a'*64, epoch=1, timeout_s=2.)
        assert not ticket.finished and value.spec.scene_manifest == ''
        assert not value.submit(CameraStateSnapshot(1, 0, 0.), ('observer',))
        for stream, state in value._processes.items():
            assert state.commands.get(timeout=1.).epoch == 1
            value._receive_message({'type':'scene_rebuilt', 'epoch':1, 'ok':True}, stream_hint=stream)
        assert value.spec.scene_manifest == ''
        ticket.wait()
        assert ticket.finished and ticket.build_s > 0 and value.spec.scene_manifest == 'new/scene.json'
        with pytest.raises(ValueError, match='no longer pending'):
            ticket.wait()
    finally:
        value.close()


def test_fixed_camera_models_reuse_descriptions_with_new_entity_handles():
    descriptions, loaded, reused = {}, [], []
    original = NS(desc=object())
    def factory():
        loaded.append(True)
        return original
    def add_resolved(*, desc):
        reused.append(desc)
        return NS(desc=desc)
    scene = NS(_sim=NS(_add_entity=add_resolved))
    assert _add_base_entity(scene, descriptions, 'robot', factory) is original
    current = _add_base_entity(scene, descriptions, 'robot', factory)
    assert current is not original and current.desc is original.desc
    assert loaded == [True] and reused == [original.desc]


def test_late_acknowledgement_is_rejected_even_if_physics_finishes_later():
    value = worker()
    try:
        ticket = value.begin_scene_rebuild('new/scene.json', 'a'*64, epoch=1, timeout_s=2.)
        for stream, state in value._processes.items():
            state.commands.get(timeout=1.)
            value._receive_message({'type':'scene_rebuilt', 'epoch':1, 'ok':True}, stream_hint=stream)
        value._edit_completed_at = ticket.deadline + .01
        with pytest.raises(TimeoutError, match='deadline'):
            ticket.wait()
        assert ticket.finished and value.spec.scene_manifest == '' and value.failure
    finally:
        value.close()
