"""Recover a lost support foot and predict stance in the future body frame."""
import numpy as np
import pytest
from scipy.integrate import quad
from scipy.spatial.transform import Rotation
from types import SimpleNamespace

from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper
from elesim_sim.robot.go2.locomotion.types import Go2Command
from elesim_sim.robot.go2.pympc_controller import (
    PyMpcGenesisController, accelerating_touchdown_candidates, stance_end_foothold,
)
from elesim_sim.robot.go2.foot_workspace import reachable
from workbench.tests.apps.sim.scenarios.go2.test_55_foot_workspace import LO, HI
from workbench.tests.apps.sim.scenarios.go2.test_59_touchdown_tracking import search_controller


@pytest.mark.parametrize('height', [.08, .094, .13])
@pytest.mark.parametrize('dt', [.01, .02])
def test_lost_stance_search_targets_ground_even_when_the_foot_bounced_high(height, dt):
    c, sample = search_controller()
    c._dt = dt
    c._last_contacts[:] = 1.  # Lost support, not a deliberately finishing swing.
    sample.feet_world[:, 2] = height
    feet = sample.feet_world.copy()
    torque = c._torques(sample, feet, np.zeros((4, 3)),
                       np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    np.testing.assert_array_equal(c._search_targets[:, :2], feet[:, :2])
    assert np.all(c._search_targets[:, 2] <= .025)
    assert np.all(c._search_min_z <= .025-.04)
    # Existing search force and actuator bounds still apply.
    assert np.all(np.linalg.norm(torque.reshape(4, 3), axis=1) <= 60.+1e-10)


def test_lost_contact_search_does_not_raise_an_already_low_foot():
    c, sample = search_controller()
    c._last_contacts[:] = 1.
    sample.feet_world[:, 2] = .021
    c._torques(sample, sample.feet_world, np.zeros((4, 3)),
               np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    assert np.all(c._search_targets[:, 2] < .021)


@pytest.mark.parametrize('rate', [-.8, 0., .8])
def test_motion_preview_follows_the_commanded_circle(rate):
    command = Go2Command(vx=.8, vy=.2, yaw_rate=rate)
    shaper = Go2CommandShaper(current=command, target=command)
    times = np.array([0., .1, .24, .5])
    positions, headings = shaper.preview_planar_motion(times)
    np.testing.assert_allclose(headings, rate*times)
    if rate:
        sine, cosine = np.sin(headings), np.cos(headings)
        expected = np.c_[.8*sine+.2*(cosine-1.), .8*(1.-cosine)+.2*sine]/rate
    else:
        expected = times[:, None]*[.8, .2]
    np.testing.assert_allclose(positions, expected, atol=1e-12)
    assert shaper.current == shaper.target == command


def test_motion_preview_integrates_simultaneous_linear_and_yaw_acceleration():
    shaper = Go2CommandShaper(current=Go2Command(vx=.2, vy=-.1, yaw_rate=-.2),
                              target=Go2Command(vx=.8, vy=.2, yaw_rate=.8))
    times = np.array([0., .1, .3, .6])
    positions, headings = shaper.preview_planar_motion(times)
    def yaw(t):
        ramp = min(t, 1./3.)
        return -.2*t+3.*ramp*(t-.5*ramp)
    def velocity(t):
        v = shaper.preview(t)
        return Rotation.from_euler('z', yaw(t)).apply([v.vx, v.vy, 0.])[:2]
    expected = np.array([[quad(lambda t: velocity(t)[i], 0., end,
                              points=[p for p in (1./3., np.hypot(.6,.3)/1.2) if p < end])[0]
                          for i in range(2)] for end in times])
    np.testing.assert_allclose(positions, expected, atol=1e-10)
    np.testing.assert_allclose(headings, [yaw(t) for t in times], atol=1e-12)
    assert shaper.current == Go2Command(vx=.2, vy=-.1, yaw_rate=-.2)


@pytest.mark.parametrize('bad', [[-.1], [float('nan')], [[.1]]])
def test_motion_preview_rejects_invalid_times(bad):
    with pytest.raises(ValueError):
        Go2CommandShaper().preview_planar_motion(bad)


def test_stance_reachability_uses_both_future_orientations(monkeypatch):
    import elesim_sim.robot.go2.pympc_controller as module
    landing = Rotation.from_euler('z', .15).as_matrix()
    end = Rotation.from_euler('z', .4).as_matrix()
    observed = []
    def project(leg, target, base, rotation, lower, upper):
        observed.append(rotation.copy())
        return np.asarray(target), True
    def can_reach(leg, target, base, rotation, lower, upper):
        observed.append(rotation.copy())
        return True
    monkeypatch.setattr(module, 'project_ground_target', project)
    monkeypatch.setattr(module, 'reachable', can_reach)
    _, feasible = stance_end_foothold(0, np.array([.2,.17,.025]),
        np.array([0.,0.,.3]), np.eye(3), np.array([.8,0.,0.]), np.array([.8,0.,0.]),
        .18, .24, LO+.04, HI-.04, predicted_base_rotations=np.array([landing,end]))
    assert feasible
    np.testing.assert_array_equal(observed, [end, landing])


def test_turning_foothold_is_reachable_in_predicted_landing_and_end_poses():
    bases = np.array([[.10,.01,.3], [.28,.06,.3]])
    rotations = Rotation.from_euler('z', [.15,.4]).as_matrix()
    result, feasible = stance_end_foothold(2, np.array([-.13,.17,.025]),
        np.array([0.,0.,.3]), np.eye(3), np.array([.8,0.,0.]), np.array([.8,0.,0.]),
        .16, .24, LO+.04, HI-.04, predicted_base_positions=bases,
        predicted_base_rotations=rotations)
    if feasible:
        for base, rotation in zip(bases, rotations):
            assert reachable(2, result, base, rotation, LO+.04, HI-.04)
    else:
        # A foot rejected at either endpoint must never be advertised as feasible.
        assert not all(reachable(2, result, base, rotation, LO+.04, HI-.04)
                       for base, rotation in zip(bases, rotations))


def test_base_travel_forecast_does_not_depend_on_world_heading_during_sideways_drift():
    from workbench.tests.apps.sim.scenarios.go2.test_68_accelerating_footholds import FEET
    command = Go2Command(vx=.8, vy=.2, yaw_rate=.8)
    shaper = Go2CommandShaper(current=command, target=command)
    def predict(rotation):
        return accelerating_touchdown_candidates([0.,0.,.3], FEET,
            yaw_rotation=rotation, velocity_world=rotation @ [.5,-.3,0.],
            command_shaper=shaper, yaw_rate=.8, remaining_swing_s=.18,
            stance_duration_s=.6/2.2, placement_scale=1.35, body_height_m=.3)
    _, landing, end = predict(np.eye(3))
    rotation = Rotation.from_euler('z', .9).as_matrix()
    _, rotated_landing, rotated_end = predict(rotation)
    np.testing.assert_allclose(rotated_landing, landing @ rotation.T, atol=1e-12)
    np.testing.assert_allclose(rotated_end, end @ rotation.T, atol=1e-12)


@pytest.mark.parametrize('current_yaw', [0., .8])
def test_blocked_support_never_forecasts_operator_yaw_while_balance_requests_zero(current_yaw):
    from workbench.tests.apps.sim.scenarios.go2.test_68_accelerating_footholds import FEET
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    c._solver = SimpleNamespace(backend='jax_mppi')
    c._config = SimpleNamespace(foot_placement_scale=1.35, z_pos_des_m=.3)
    c._contact_gait = SimpleNamespace(blocked=True)
    c._nominal_foot_offsets_body = FEET.copy()
    c._command_shaper = Go2CommandShaper(
        current=Go2Command(vx=.8,yaw_rate=current_yaw), target=Go2Command(vx=.8,yaw_rate=.8))
    rotation = Rotation.from_euler('xyz', [.1,-.1,.7]).as_matrix()
    _, _, future_rotations = c._predict_touchdowns([0.,0.,.3],
        Rotation.from_euler('z',.7).as_matrix(), [.5,.4,0.], np.zeros(3), [.8,0.,0.],
        0., np.full(4,.18), .6/2.2, base_rotation=rotation)
    np.testing.assert_allclose(future_rotations, np.broadcast_to(rotation,(4,2,3,3)),atol=1e-12)
    assert c._command_shaper.current.yaw_rate == current_yaw
    assert c._command_shaper.target.yaw_rate == .8
