"""Recover body attitude on existing support before lifting the next pair."""
import numpy as np
import pytest

from elesim_sim.robot.go2 import pympc_controller as control
from workbench.tests.apps.sim.scenarios.go2.test_61_support_handover import gait_at, advance


@pytest.mark.parametrize('rpy,angular', [
    ([0., 0., 2.8], [0., 0., .8]),
    ([.12, -.1, -2.8], [.2, -.2, .8]),
])
def test_normal_trot_and_level_yaw_remain_ready(rpy, angular):
    assert control.body_ready_for_liftoff(rpy, angular)


@pytest.mark.parametrize('rpy,angular', [
    ([.50261686062, .1531284544, -1.33027197], [.16023, -1.09456, .40719]),
    ([.26, 0., 0.], [0., 0., 0.]),
    ([0., -.26, 0.], [0., 0., 0.]),
    ([0., 0., 0.], [1.01, 0., 0.]),
    ([0., 0., 0.], [0., -1.01, 0.]),
])
def test_large_tilt_or_tilt_rate_needs_four_foot_recovery(rpy, angular):
    assert not control.body_ready_for_liftoff(rpy, angular)


def test_tilt_rate_uses_euler_kinematics_including_body_yaw_coupling():
    # Body pitch rate alone is below 1 rad/s, but the Euler pitch rate is
    # cos(roll)*wy - sin(roll)*wz, which exceeds it at this inclined turn.
    assert not control.body_ready_for_liftoff([.2, 0., 0.], [0., .9, -1.])


@pytest.mark.parametrize('rpy,angular', [([0., 0.], [0., 0., 0.]),
                                      ([0., np.nan, 0.], [0., 0., 0.]),
                                      ([0., 0., 0.], [0., np.inf, 0.])])
def test_invalid_body_feedback_is_rejected(rpy, angular):
    with pytest.raises(ValueError, match='liftoff'):
        control.body_ready_for_liftoff(rpy, angular)


def test_posture_hold_keeps_current_support_and_reports_blocked_until_recovered():
    gate = gait_at(.72)
    args = dict(foot_heights=np.full(4, .023), dt=.01, gait_hz=2.2,
                duty=.6, horizon_dt=.02, horizon=12, settling=False,
                support_handover=True)
    for _ in range(5):
        schedule = gate.update(np.full(4, 60.), liftoff_ready=False, **args)
        assert gate.time == pytest.approx(.72)
        assert gate.blocked
        assert gate.posture_hold
        assert not gate.search.any()
        assert not gate._swinging.any()
        np.testing.assert_array_equal(schedule, np.ones((4, 12)))
    gate.update(np.full(4, 60.), liftoff_ready=True, **args)
    assert gate.time == pytest.approx(.73)
    assert not gate.blocked
    assert not gate.posture_hold
    assert gate._swinging[[0, 3]].all()


def test_lost_present_load_also_marks_the_liftoff_hold_as_blocked():
    gate = gait_at(.72)
    advance(gate, [60., 0., 60., 60.], np.full(4, .023))
    assert gate.time == pytest.approx(.72)
    assert gate.blocked


def test_posture_hold_cannot_pause_a_swing_that_has_already_started():
    gate = gait_at(.72)
    advance(gate, np.full(4, 60.), np.full(4, .023))
    first = gate.swing_elapsed.copy()
    gate.update(np.full(4, 60.), foot_heights=np.full(4, .023), dt=.01,
                gait_hz=2.2, duty=.6, horizon_dt=.02, horizon=12,
                settling=False, support_handover=True, liftoff_ready=False)
    np.testing.assert_allclose(gate.swing_elapsed[[0, 3]], first[[0, 3]] + .01)


def test_cpu_legacy_gate_retains_its_original_liftoff_policy():
    gate = gait_at(.72)
    gate.update(np.full(4, 60.), foot_heights=np.full(4, .023), dt=.01,
                gait_hz=2.2, duty=.6, horizon_dt=.02, horizon=12,
                settling=False, support_handover=False, liftoff_ready=False)
    assert gate.time == pytest.approx(.73)


@pytest.mark.parametrize('posture_hold', [False, True])
def test_only_posture_recovery_brakes_translation_instead_of_retaining_progress(posture_hold):
    from dataclasses import replace
    from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
    c, state = controller()
    c._contact_gait.blocked = True
    c._contact_gait.posture_hold = posture_hold
    c._solver.horizon, c._solver.dt = 12, .02
    state = replace(state, desired_com_velocity=np.zeros(3))
    actual = c._walking_reference(state)
    np.testing.assert_array_equal(actual.desired_com_velocity,
                                  np.zeros(3) if posture_hold else [.3, 0., 0.])


def test_posture_recovery_retains_the_actual_stance_braking_damping():
    from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
    c, state = controller()
    c._contact_gait.posture_hold = True
    reference = c._stance_damping_reference(state, np.tile(np.eye(3), (4, 1, 1)))
    np.testing.assert_array_equal(reference, np.zeros(12))
