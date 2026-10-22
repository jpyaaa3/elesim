"""Footholds must anticipate the shaped command and the rising ready body."""
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper
from elesim_sim.robot.go2.locomotion.types import Go2Command
from elesim_sim.robot.go2.pympc_controller import (
    PyMpcGenesisController, accelerating_touchdown_candidates,
    stance_end_foothold, touchdown_candidates_world, touchdown_offsets_body,
)
from elesim_sim.robot.go2.foot_workspace import reachable
from workbench.tests.apps.sim.scenarios.go2.test_55_foot_workspace import LO, HI


@pytest.mark.parametrize('current,target,dt,expected_v,expected_x', [
    (Go2Command(), Go2Command(vx=.8), .2, [.24, 0., 0.], [.024, 0.]),
    (Go2Command(vx=.8), Go2Command(), .2, [.56, 0., 0.], [.136, 0.]),
    (Go2Command(vx=.2), Go2Command(vx=-.8), .3, [-.16, 0., 0.], [.006, 0.]),
    (Go2Command(), Go2Command(vx=.8), 1., [.8, 0., 0.], [.8- .8**2/2.4, 0.]),
    (Go2Command(), Go2Command(vx=.6, vy=.8), .25, [.18, .24, 0.], [.0225, .03]),
    (Go2Command(vx=.3, vy=-.2, yaw_rate=.1),
     Go2Command(vx=.3, vy=-.2, yaw_rate=.7), .5, [.3, -.2, .7], [.15, -.1]),
])
def test_preview_integrates_acceleration_braking_reversal_and_saturation(
        current, target, dt, expected_v, expected_x):
    s = Go2CommandShaper(current=current, target=target)
    s._zero_since_s = 3.
    v = s.preview(dt)
    np.testing.assert_allclose([v.vx, v.vy, v.yaw_rate], expected_v, atol=1e-12)
    np.testing.assert_allclose(s.preview_planar_displacement(dt), expected_x, atol=1e-12)
    assert (s.current, s.target, s._zero_since_s) == (current, target, 3.)
    assert s.update(dt) == v


@pytest.mark.parametrize('dt', [-.1, float('nan'), float('inf')])
def test_invalid_preview_time_is_rejected(dt):
    s = Go2CommandShaper()
    with pytest.raises(ValueError): s.preview(dt)
    with pytest.raises(ValueError): s.preview_planar_displacement(dt)


FEET = np.array([[.1934, .178, -.27], [.1934, -.178, -.27],
                 [-.1934, .178, -.27], [-.1934, -.178, -.27]])


def prediction(*, sign=1., yaw=0., current=.168, target=.8, speed=.05, height=.273,
               remaining=None):
    rotation = Rotation.from_euler('z', yaw).as_matrix()
    shaper = Go2CommandShaper(current=Go2Command(vx=sign*current),
                              target=Go2Command(vx=sign*target))
    times = np.full(4, .18) if remaining is None else np.asarray(remaining)
    return accelerating_touchdown_candidates(
        np.array([0., 0., height]), FEET, yaw_rotation=rotation,
        velocity_world=rotation @ np.array([sign*speed, 0., 0.]),
        command_shaper=shaper, yaw_rate=0., remaining_swing_s=times,
        stance_duration_s=.6/2.2, placement_scale=1.35, body_height_m=.30,
    )


def test_next_foot_anticipates_acceleration_and_restored_body_height():
    candidates, landing, end = prediction()
    s = Go2CommandShaper(current=Go2Command(vx=.168), target=Go2Command(vx=.8))
    np.testing.assert_allclose(landing[:, 0], s.preview_planar_displacement(.18)[0])
    np.testing.assert_allclose(end[:, 0], s.preview_planar_displacement(.18+.6/2.2+.06)[0])
    np.testing.assert_allclose(landing[:, 2], .30)
    np.testing.assert_allclose(end[:, 2], .30)
    old = touchdown_candidates_world(
        [0., 0., .273], touchdown_offsets_body(FEET, command_body=[.168, 0., 0.],
        yaw_rate=0., half_stance_s=.3/2.2, placement_scale=1.35),
        yaw_rotation=np.eye(3), velocity_world=[.05, 0., 0.],
        desired_velocity_world=[.168, 0., 0.], remaining_swing_s=.18,
    )
    assert np.all(candidates[:, 0] > old[:, 0]+.05)


@pytest.mark.parametrize('sign,leg', [(1., 2), (-1., 0)])
def test_accelerating_trailing_leg_is_reachable_at_landing_and_support_end(sign, leg):
    candidates, landing, end = prediction(sign=sign)
    result, feasible = stance_end_foothold(
        leg, candidates[leg], [0., 0., .273], np.eye(3), [sign*.05, 0., 0.],
        [sign*.168, 0., 0.], .18, .6/2.2, LO+.04, HI-.04,
        predicted_base_positions=(landing[leg], end[leg]),
    )
    assert feasible
    assert reachable(leg, result, landing[leg], np.eye(3), LO+.04, HI-.04)
    assert reachable(leg, result, end[leg], np.eye(3), LO+.04, HI-.04)
    assert result[2] == pytest.approx(.025)


def test_zero_height_forecast_does_not_lower_an_already_higher_body():
    _, landing, end = prediction(height=.33)
    np.testing.assert_allclose(landing[:, 2], .33)
    np.testing.assert_allclose(end[:, 2], .33)


def test_prediction_rotates_with_heading_and_does_not_modify_nominal_width():
    plain, landing, end = prediction()
    yaw = .7
    rot = Rotation.from_euler('z', yaw).as_matrix()
    turned, turned_landing, turned_end = prediction(yaw=yaw)
    np.testing.assert_allclose(turned, plain @ rot.T, atol=1e-12)
    np.testing.assert_allclose(turned_landing, landing @ rot.T, atol=1e-12)
    np.testing.assert_allclose(turned_end, end @ rot.T, atol=1e-12)
    np.testing.assert_allclose(plain[:, 1], FEET[:, 1])


def test_acceleration_forecast_preserves_the_established_cruise_stride():
    # GPU2 bf2b36a: expanding the steady lead by another 60 ms raised
    # backward pitch to 14 deg and cycle3 fell. Preserve the previously
    # stable cruise placement; extra load time belongs to the reach check.
    candidates, _, _ = prediction(current=.8, target=.8, speed=.8)
    expected = FEET[:, 0] + .8*.18 + .8*.5*(.6/2.2)*1.35
    np.testing.assert_allclose(candidates[:, 0], expected)


def test_each_future_recontact_uses_its_own_acceleration_interval():
    times = np.array([0., .1, .2, .3])
    candidates, landing, end = prediction(remaining=times)
    assert np.all(np.diff(landing[:, 0]) > 0.)
    assert np.all(np.diff(end[:, 0]) > 0.)
    assert candidates[3, 0]-FEET[3, 0] > candidates[0, 0]-FEET[0, 0]+.1


def test_combined_velocity_and_acceleration_travel_remains_bounded():
    s = Go2CommandShaper(current=Go2Command(vx=.3, vy=.4),
                          target=Go2Command(vx=.6, vy=.8))
    kwargs = dict(yaw_rotation=np.eye(3), velocity_world=[3., 4., 0.],
                  command_shaper=s, yaw_rate=0., stance_duration_s=.6/2.2,
                  placement_scale=1.35, body_height_m=.3)
    a, *_ = accelerating_touchdown_candidates([0.,0.,.3], FEET,
                         remaining_swing_s=np.ones(4), **kwargs)
    half = .5*(.6/2.2)
    lead = np.subtract(s.preview_planar_displacement(1.+half),
                       s.preview_planar_displacement(1.))*1.35
    correction = np.clip(.1*(np.array([3.,4.])-np.array([.3,.4])), -.04,.04)
    np.testing.assert_allclose(np.linalg.norm(a[:,:2]-FEET[:,:2]-lead-correction, axis=1), .15)


@pytest.mark.parametrize('backend', ['jax_mppi', 'acados'])
def test_controller_routes_both_liftoff_and_future_preview_through_one_prediction(backend):
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    c._solver = SimpleNamespace(backend=backend)
    c._config = SimpleNamespace(foot_placement_scale=1.35, z_pos_des_m=.3)
    c._nominal_foot_offsets_body = FEET.copy()
    c._command_shaper = Go2CommandShaper(current=Go2Command(vx=.168),
                                       target=Go2Command(vx=.8))
    times = np.array([.18, .27, 0., .09])
    candidates, bases, rotations = c._predict_touchdowns(
        [0.,0.,.273], np.eye(3), [.05,0.,0.], [.168,0.,0.],
        [ .168,0.,0.], 0., times, .6/2.2)
    if backend == 'jax_mppi':
        expected, landing, end = prediction(remaining=times)
        np.testing.assert_allclose(candidates, expected)
        np.testing.assert_allclose(bases[:,0], landing)
        np.testing.assert_allclose(bases[:,1], end)
        np.testing.assert_array_equal(rotations, np.broadcast_to(np.eye(3), (4, 2, 3, 3)))
    else:
        assert bases is None
        assert rotations is None
        expected = touchdown_candidates_world([0.,0.,.273],
            touchdown_offsets_body(FEET, command_body=[.168,0.,0.], yaw_rate=0.,
                half_stance_s=.3/2.2, placement_scale=1.35),
            yaw_rotation=np.eye(3), velocity_world=[.05,0.,0.],
            desired_velocity_world=[.168,0.,0.], remaining_swing_s=times)
        expected[:, 2] = .025  # The CPU controller's existing flat floor.
        np.testing.assert_array_equal(candidates, expected)


def sample_controller():
    from pathlib import Path
    from unittest.mock import Mock
    import pinocchio as pin
    from elesim_sim.robot.go2.mpc.config import Go2MpcConfig
    from elesim_sim.robot.go2.pympc_controller import contact_schedule
    root = next(p for p in Path(__file__).resolve().parents if (p/'payload').is_dir())
    model = pin.buildModelFromUrdf(
        str(root/'payload/data/models/assemblies/zed-mini/assets/go2/go2.urdf'),
        pin.JointModelFreeFlyer())
    q = pin.neutral(model)
    q[:3] = [0., 0., .273]
    q[7:] = np.tile([0., .9, -1.8], 4)
    dq = np.r_[.05, np.zeros(model.nv-1)]
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    c._pin, c._model, c._data = pin, model, model.createData()
    c._frames = [model.getFrameId(leg+'_foot') for leg in ('FL','FR','RL','RR')]
    c._bridge = SimpleNamespace(read_pin_q_dq=lambda **kw: (q, dq))
    c._config, c._payload = Go2MpcConfig(), None
    c._solver = SimpleNamespace(backend='jax_mppi', horizon=30, dt=.02)
    c._command_scale = lambda: 1.
    c._normal_contact_forces = lambda: np.full(4, 40.)
    schedule = contact_schedule(.07, gait_hz=c._config.gait_hz,
                                duty=c._config.gait_duty, dt=.02, horizon=30)
    c._contact_gait = SimpleNamespace(update=lambda *args, **kw: schedule.copy(),
        blocked=False, load=schedule[:,0].copy(), search=np.zeros(4,dtype=bool),
        swing_remaining=np.where(schedule[:,0] == 0., .15, 0.), time=.07)
    c._cmd = Go2Command(vx=.168)
    c._command_shaper = Go2CommandShaper(current=c._cmd, target=Go2Command(vx=.8))
    c._dt, c._sim_time, c._walk_started_s, c._settling = .01, 1., 0., False
    c._step_i, c._solve_stride = 0, 2
    c._last_contacts = np.ones(4)
    c._touchdowns = FEET.copy()
    c._was_searching = np.zeros(4,dtype=bool)
    c._nominal_foot_offsets_body = FEET.copy()
    c._foot_lower, c._foot_upper = np.tile(LO+.04,(4,1)), np.tile(HI-.04,(4,1))
    c._predict_touchdowns = Mock(wraps=c._predict_touchdowns)
    return c, q, schedule


def test_actual_sample_predicts_liftoff_and_next_stance_but_keeps_a_latched_swing():
    c, q, schedule = sample_controller()
    first, *_ = c._sample()
    assert c._predict_touchdowns.call_count == 2
    assert np.any(first.preview_footholds_world != first.footholds_world)
    swinging = schedule[:,0] == 0.
    assert swinging.any()
    c._last_contacts = schedule[:,0].copy()
    c._touchdowns = first.footholds_world.copy()
    q[0] += .05
    c._cmd = c._command_shaper.update(.1)
    second, *_ = c._sample()
    np.testing.assert_array_equal(second.footholds_world[swinging], first.footholds_world[swinging])
    assert c._predict_touchdowns.call_count == 3


def test_between_solves_sample_retains_actuation_state_without_unused_foothold_predictions(monkeypatch):
    from dataclasses import fields
    c, q, schedule = sample_controller()
    first, *_ = c._sample()
    c._last_contacts = schedule[:,0].copy()
    c._touchdowns = first.footholds_world.copy()
    c._step_i = 1
    c._predict_touchdowns.reset_mock()
    actual = c._sample()
    assert c._predict_touchdowns.call_count == 0
    with monkeypatch.context() as patch:
        patch.setattr(c,'_force_solve_due',lambda contacts:True)
        reference = c._sample()
    for field in fields(actual[0]):
        if field.name != 'preview_footholds_world':
            np.testing.assert_equal(getattr(actual[0],field.name),getattr(reference[0],field.name))
    for measured, expected in zip(actual[1:],reference[1:]):
        np.testing.assert_array_equal(measured,expected)
    assert np.any(reference[0].preview_footholds_world!=reference[0].footholds_world)
    # A support change must prepare a new force solve even on an odd tick.
    c._last_contacts[np.flatnonzero(schedule[:,0])[0]] = 0.
    c._predict_touchdowns.reset_mock()
    support_changed, *_ = c._sample()
    assert c._predict_touchdowns.call_count == 1
    np.testing.assert_array_equal(support_changed.preview_footholds_world,
                                  reference[0].preview_footholds_world)
