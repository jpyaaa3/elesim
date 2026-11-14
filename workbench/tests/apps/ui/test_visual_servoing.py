"""Operator intent gates, lifecycle correlation and truthful reconstruction UX."""
from types import SimpleNamespace as NS

import pytest

from elesim_protocol import MockObjectStatePayload, SimulationStatusPayload
from elesim_ui.visual_servoing import ServoingController
from workbench.tools.ui_preview import preview_panel


@pytest.fixture
def ready():
    panel, cache = preview_panel()
    cache.status = NS(pilot_online=True)
    cache.services.update(active_endpoint='sim-preview', has_client=True,
        available_endpoints=[dict(endpoint_id='sim-preview', instance_id='boot-a', role='sim',
            capabilities=['simulation.mock_hug.v1'])],
        current_host_state=NS(connected=True, torque_enabled=True, safety_fault=''),
        object_scan=dict(phase='idle', active=False, can_start=True))
    session = panel._sim_view.session
    session.active_sim_id, session.active_sim_boot_id = 'sim-preview', 'boot-a'
    session.supports_scene_rebuild = True
    session.snapshot = NS(active_sim_id='sim-preview', session_id='session-a',
        requested_sim_id='sim-preview', pending_commands=0, last_result=None, last_error='',
        status=SimulationStatusPayload(2, False, 1., False, 0., mock_object=MockObjectStatePayload(
            available_assets=('jar', 'demo_box'), state='spawned', asset_id='jar', revision=1)))
    controller = ServoingController(panel)
    return controller, panel, cache, session


def names(cache):
    return [request[0][1] for request in cache.requests if isinstance(request[0], tuple)]


def complete(cache, *, ok=True, value=None):
    kwargs = cache.requests[-1][1]
    kwargs['on_result' if ok else 'on_error'](value)


def test_constructing_and_reading_workspace_never_sends_intents(ready):
    controller, panel, cache, _ = ready
    cache.requests.clear()
    for _ in range(5):
        controller.refresh()
        controller.view()
        for action in ('scan', 'spawn', 'remove', 'compute_hug', 'execute_hug', 'apply', 'wrap'):
            controller.reason(action)
    assert cache.requests == []


def test_scan_is_one_explicit_pilot_intent_and_duplicate_click_is_suppressed(ready):
    c, _, cache, _ = ready
    c.target_id, c.max_views = 'apple-1', 8
    assert c.perform('scan')
    assert not c.perform('scan')
    assert names(cache) == ['start_object_scan']
    assert cache.requests[-1][0][2:] == ('apple-1', 8)
    assert c.step == 1


@pytest.mark.parametrize('target,views', [('',12),('x'*65,12),('ok',1),('ok',25)])
def test_bad_scan_drafts_never_leave_ui(ready, target, views):
    c, _, cache, _ = ready
    c.target_id, c.max_views = target, views
    assert not c.perform('scan')
    assert not cache.requests


@pytest.mark.parametrize('failure', ['offline', 'robot', 'busy', 'pilot_blocked'])
def test_scan_uses_live_route_and_authoritative_pilot_preflight(ready, failure):
    c, _, cache, _ = ready
    if failure == 'offline':
        cache.status.pilot_online = False
    elif failure == 'robot':
        cache.services['available_endpoints'][0]['role'] = 'robot'
    elif failure == 'busy':
        cache.values['gaze_running'] = True
    else:
        cache.services['object_scan'].update(can_start=False, blocked_reason='open claws first')
    assert c.reason('scan')
    assert not c.perform('scan')
    assert not cache.requests


def test_cancel_survives_offline_preflight_and_pending_start(ready):
    c, _, cache, _ = ready
    assert c.perform('scan')
    cache.status.pilot_online = False
    assert c.perform('cancel_scan')
    assert names(cache) == ['start_object_scan', 'cancel_object_scan']


@pytest.mark.parametrize('change', ['boot', 'route', 'epoch', 'revision', 'ui_session'])
def test_late_hug_solution_is_ignored_after_context_changes(ready, change):
    c, _, cache, session = ready
    assert c.perform('compute_hug')
    callback = cache.requests[-1][1]['on_result']
    if change == 'boot':
        cache.services['available_endpoints'][0]['instance_id'] = 'boot-b'
    elif change == 'route':
        cache.services['active_endpoint'] = 'robot-other'
    elif change == 'epoch':
        session.snapshot.status = SimulationStatusPayload(3, False, 1., False, 0.)
    elif change == 'ui_session':
        session.snapshot.session_id = 'session-b'
    else:
        session.snapshot.status = SimulationStatusPayload(2,False,1.,False,0.,
            mock_object=MockObjectStatePayload(state='spawned', asset_id='jar', revision=2))
    callback({'solution_id':'stale'})
    assert not c.solution
    assert c.reason('execute_hug')
    assert not c.perform('execute_hug')
    assert names(cache) == ['compute_mock_hug']


def test_mock_scene_commands_and_hug_motion_keep_separate_authorities(ready):
    c, _, cache, session = ready
    scene_commands = []
    session.send_command = lambda name, args=None: scene_commands.append((name,args)) or 'scene-1'
    c.asset_id = 'jar'
    assert c.perform('spawn')
    assert scene_commands == [('spawn_mock_object', dict(asset_id='jar',
        position=[.5,0.,.4],euler_deg=[0.,0.,0.]))]
    assert not cache.requests
    session.snapshot.last_result = NS(request_id='scene-1', ok=True, reason='spawned')
    c.refresh()
    assert c.perform('compute_hug')
    complete(cache, value={'solution_id':'solution-a'})
    assert c.perform('execute_hug')
    assert names(cache) == ['compute_mock_hug','execute_mock_hug']
    assert cache.requests[-1][0][2:] == ('solution-a',)


@pytest.mark.parametrize('action', ['spawn','remove','detach','apply','wrap','compute_hug'])
def test_running_scan_blocks_scene_changes_and_competing_motion(ready, action):
    c, _, cache, _ = ready
    cache.services['object_scan'].update(active=True,phase='moving',can_start=False)
    assert c.reason(action)
    assert not c.perform(action)
    assert not cache.requests


def test_apply_uses_published_revision_and_only_its_matching_ack(ready):
    c, _, cache, session = ready
    scan = dict(phase='completed',active=False,job_id='job-a', sim_id='sim-preview',
        sim_boot_id='boot-a',scene_revision='sha-a',scene_reference='scan-a/scene.json')
    cache.services['object_scan'] = scan
    calls = []
    session.load_scan_result = lambda status: calls.append(status) or 'apply-1'
    assert c.perform('apply')
    assert calls == [scan]
    session.snapshot.last_result = NS(request_id='other',ok=True,reason='unrelated ACK')
    c.refresh()
    assert c.scene_outcome != 'applied'
    session.snapshot.last_result = NS(request_id='apply-1',ok=False,reason='revision mismatch')
    c.refresh()
    assert c.scene_outcome == 'failed' and 'revision mismatch' in c.error
    assert c.perform('apply')
    session.snapshot.last_result = NS(request_id='apply-1',ok=True,reason='scene rebuilt')
    c.refresh()
    assert c.scene_outcome == 'applied'
    assert not c.perform('apply')  # No repeated expensive rebuild for the same result.


@pytest.mark.parametrize('field,value', [('sim_id','sim-other'),('sim_boot_id','old'),
    ('scene_revision',''),('scene_reference',''),('active',True)])
def test_cannot_apply_foreign_or_unpublished_scan(ready, field, value):
    c, _, cache, _ = ready
    cache.services['object_scan'] = dict(phase='completed',active=False,sim_id='sim-preview',
        sim_boot_id='boot-a',scene_revision='sha-a',scene_reference='scan-a/scene.json',**{})
    cache.services['object_scan'][field] = value
    assert c.reason('apply')
    assert not c.perform('apply')
    assert not cache.requests


def test_mesh_grasp_is_never_implemented_by_truth_hug_or_configured_pole_policy(ready):
    c, _, cache, _ = ready
    for action in ('gripper','embrace','verify'):
        assert 'adapter' in c.reason(action).lower()
        assert not c.perform(action)
    assert not cache.requests


def test_failed_or_timed_out_command_can_retry_and_old_callback_cannot_override(ready):
    c, _, cache, _ = ready
    assert c.perform('compute_hug')
    old = cache.requests[-1][1]['on_result']
    c.refresh(now=10**12)
    assert 'timed out' in c.error
    assert c.perform('compute_hug')
    old({'solution_id':'expired'})
    assert not c.solution
    complete(cache, ok=False, value='model missing')
    assert 'model missing' in c.error
    assert c.perform('compute_hug')


def test_complete_means_reconstructed_and_coverage_never_means_hidden_surface_complete(ready):
    c, _, cache, _ = ready
    cache.services['object_scan'].update(phase='completed',can_start=True,
        needs_more_observation=True,coverage={'fraction':1.})
    v = c.view()
    assert v.reconstructed
    assert 'unverified' in v.coverage_note
    assert not v.grasp_verified


def test_spawn_needs_ui_sim_authority_but_not_pilot_motion_authority(ready):
    c, _, cache, session = ready
    cache.status.pilot_online = False
    cache.services.update(has_client=False,active_endpoint='robot-other')
    c.asset_id = 'jar'
    calls = []
    session.send_command = lambda *args:calls.append(args) or 'scene-a'
    assert c.perform('spawn')
    assert calls[0][0] == 'spawn_mock_object' and not cache.requests


def test_successful_request_without_an_accepted_operation_is_not_reported_as_started(ready):
    c, _, cache, _ = ready
    assert c.perform('wrap')
    complete(cache,value=False)
    assert c.error and 'accepted' not in c.notice.lower()


def test_stops_are_independent_and_pending_motion_stays_cancellable(ready):
    c, panel, cache, session = ready
    assert c.perform('wrap')
    c.stop()
    assert 'stop_pick_e2e' in names(cache)
    cache.services['wrap_grasp_running'] = True
    cache.services['arm_trajectory'] = {'active':True}
    cache.values['gaze_running'] = True
    calls = []
    def fail():
        calls.append('trajectory')
        raise RuntimeError('cannot stop trajectory')
    panel.service.cancel_arm_trajectory = fail
    panel.service.stop_pick_e2e = lambda:calls.append('pick')
    panel.service.stop_gaze_stabilizer = lambda:calls.append('gaze')
    c.stop()
    assert calls == ['trajectory','pick','gaze']
    assert 'cannot stop trajectory' in c.error


def test_in_flight_cancel_cannot_be_spammed(ready):
    c, _, cache, _ = ready
    cache.services['object_scan']['active'] = True
    assert c.perform('cancel_scan')
    assert not c.perform('cancel_scan')
    assert names(cache) == ['cancel_object_scan']


def test_pending_reference_hug_stops_via_ui_sim_pause_without_mock_hold_claim(ready):
    c, _, cache, session = ready
    assert c.perform('compute_hug')
    complete(cache,value={'solution_id':'hug-a'})
    assert c.perform('execute_hug')
    calls = []
    session.send_command = lambda *args:calls.append(args) or 'pause-a'
    c.stop()
    assert calls == [('pause',)]
    assert not c.solution and not c.view().grasp_verified
    assert 'pausing Sim' in c.notice


def test_global_stop_uses_torque_off_for_non_cancellable_ik_solver(ready):
    c, _, cache, _ = ready
    cache.values['ik_running'] = True
    c.stop()
    assert names(cache) == ['torque_off']
    assert 'no cancellation adapter' in c.notice


def test_scene_ack_remains_valid_when_only_motion_target_changes(ready):
    c, _, cache, session = ready
    c.asset_id = 'jar'
    session.send_command = lambda *_:'spawn-independent'
    assert c.perform('spawn')
    cache.services['active_endpoint'] = 'robot-other'
    session.snapshot.last_result = NS(request_id='spawn-independent',ok=True,reason='placed')
    c.refresh()
    assert not c.pending and not c.error and c.notice == 'placed'


@pytest.mark.parametrize('action',['scan','wrap'])
def test_pilot_intent_ack_does_not_depend_on_camera_view_selection(ready, action):
    c, _, cache, session = ready
    assert c.perform(action)
    session.active_sim_id = 'sim-other'
    session.snapshot.session_id = 'other-camera-session'
    c.refresh()
    complete(cache,value=True)
    assert not c.pending and not c.error
    assert 'accepted' in c.notice


def test_apply_ack_and_badge_follow_sim_session_not_motion_target(ready):
    c, _, cache, session = ready
    cache.services['object_scan'] = dict(phase='completed',active=False,job_id='scan-a',
        sim_id='sim-preview',sim_boot_id='boot-a',scene_reference='scan/scene.json',scene_revision='sha')
    session.load_scan_result = lambda _:'apply-independent'
    assert c.perform('apply')
    cache.services['active_endpoint'] = 'robot-other'
    session.snapshot.last_result = NS(request_id='apply-independent',ok=True,reason='applied')
    c.refresh()
    assert c.scene_outcome == 'applied' and not c.error
    assert 'already applied' in c.reason('apply')


def test_apply_ack_is_discarded_when_its_ui_sim_session_changes(ready):
    c, _, cache, session = ready
    cache.services['object_scan'] = dict(phase='completed',active=False,job_id='scan-a',
        sim_id='sim-preview',sim_boot_id='boot-a',scene_reference='scan/scene.json',scene_revision='sha')
    session.load_scan_result = lambda _:'apply-old-session'
    assert c.perform('apply')
    session.snapshot.session_id = 'session-new'
    session.snapshot.last_result = NS(request_id='apply-old-session',ok=True,reason='applied')
    c.refresh()
    assert c.scene_outcome != 'applied' and c.error


@pytest.mark.parametrize('action',['scan','wrap'])
def test_another_arm_workflow_invalidates_previously_computed_reference_solution(ready,action):
    c, _, cache, _ = ready
    assert c.perform('compute_hug')
    complete(cache,value={'solution_id':'before-motion'})
    assert c.perform(action)
    complete(cache,value=True)
    assert not c.solution
    assert 'fresh solution' in c.reason('execute_hug')


def test_paused_sim_can_resume_with_only_ui_authority_even_if_pilot_is_offline(ready):
    from dataclasses import replace
    c, _, cache, session = ready
    session.snapshot.status = replace(session.snapshot.status,paused=True)
    session.supports_scene_rebuild = False  # Basic Sim controls predate scene rebuild.
    cache.status.pilot_online = False
    cache.services.update(active_endpoint='robot-other',wrap_grasp_running=True)
    calls = []
    session.send_command = lambda *args:calls.append(args) or 'resume-a'
    c.refresh()
    assert not calls
    assert c.perform('resume_sim')
    assert calls == [('resume',None)] and not cache.requests
    session.snapshot.last_result = NS(request_id='resume-a',ok=True,reason='running')
    c.refresh()
    assert not c.error and c.notice == 'running'


def test_running_sim_does_not_receive_redundant_resume_intents(ready):
    c, _, cache, session = ready
    session.send_command = lambda *_:pytest.fail('running Sim must not receive resume')
    assert not c.perform('resume_sim')
    assert not cache.requests


def test_applied_badge_is_invalidated_by_manual_scene_or_sim_reset(ready):
    c, _, cache, session = ready
    cache.services['object_scan'] = dict(phase='completed',active=False,job_id='job-a',
        sim_id='sim-preview',sim_boot_id='boot-a',scene_reference='scan/scene.json',scene_revision='sha')
    session.load_scan_result = lambda _: 'apply-a'
    assert c.perform('apply')
    session.snapshot.status = SimulationStatusPayload(3,False,1.,False,0.)
    session.snapshot.last_result = NS(request_id='apply-a',ok=True,reason='applied')
    c.refresh()
    assert c.scene_outcome == 'applied' and c.reason('apply')
    session.send_command = lambda *_:'clear-a'
    assert c.perform('clear_scene')
    session.snapshot.last_result = NS(request_id='clear-a',ok=True,reason='cleared')
    c.refresh()
    assert not c.reason('apply')
    assert c.perform('apply')
    session.snapshot.status = SimulationStatusPayload(4,False,1.,False,0.)
    session.snapshot.last_result = NS(request_id='apply-a',ok=True,reason='applied')
    c.refresh()
    session.snapshot.status = SimulationStatusPayload(5,False,1.,False,0.)
    c.refresh()
    assert c.scene_outcome != 'applied' and not c.reason('apply')


def test_reconstructed_hug_prepares_then_executes_once_without_coordinates(ready):
    c, _, cache, _ = ready
    cache.services['object_scan'].update(phase='completed',job_id='scan-a',scene_revision='rev-a')
    assert c.perform('plan_reconstructed_hug')
    assert names(cache)==['plan_reconstructed_hug']
    assert cache.requests[-1][0][2:]==('target',)
    complete(cache,value={'plan_id':'hug-a','weights':[.2,.3,.5]})
    assert c.perform('execute_reconstructed_hug')
    assert names(cache)[-1]=='execute_arm_trajectory'
    assert cache.requests[-1][0][2:]==('hug-a',)
    assert not c.hug_plan


def test_new_reconstruction_invalidates_prepared_hug(ready):
    c, _, cache, _ = ready
    cache.services['object_scan'].update(phase='completed',job_id='scan-a',scene_revision='rev-a')
    assert c.perform('plan_reconstructed_hug')
    complete(cache,value={'plan_id':'hug-a'})
    cache.services['object_scan']['scene_revision']='rev-b'
    assert not c.perform('execute_reconstructed_hug')
    assert names(cache)==['plan_reconstructed_hug']
