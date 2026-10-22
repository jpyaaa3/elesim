"""Overlap nominal four-foot dwell with measured landing confirmation."""
import numpy as np
import pytest

from workbench.tests.apps.sim.scenarios.go2.test_61_support_handover import gait_at, advance


@pytest.mark.parametrize('incoming', [[0, 3], [1, 2]])
def test_nominal_overlap_can_progress_while_one_diagonal_landing_ramps(incoming):
    # Both phases start safely inside the all-four interval, before liftoff.
    time = 5.01 if incoming == [0, 3] else .69
    gate = gait_at(time)
    gate.load[incoming] = .5
    gate.blocked = True
    schedule = advance(gate, np.full(4, 50.), np.full(4, .023))
    assert gate.time == pytest.approx(time+.01)
    assert gate.blocked
    assert not gate._swinging.any()
    np.testing.assert_array_equal(schedule, np.ones((4, 12)))


def test_advancing_overlap_still_stops_at_next_liftoff_until_actual_load_is_complete():
    gate = gait_at(5.01)
    gate.confirmed[[0, 3]] = False
    gate.load[[0, 3]] = 0.
    gate.blocked = True
    forces = np.array([0., 60., 60., 0.])
    heights = np.array([.04, .023, .023, .04])
    for _ in range(8):
        schedule = advance(gate, forces, heights)
        assert not gate._swinging.any()
        np.testing.assert_array_equal(schedule[[1, 2]], np.ones((2, 12)))
        np.testing.assert_array_equal(schedule[[0, 3]], np.zeros((2, 12)))
    assert gate.time == pytest.approx(5.04)
    assert gate.phase_rate == 0.
    # Continue to respect the entire real contact confirmation and load ramp.
    for _ in range(8):
        advance(gate, np.full(4, 60.), np.full(4, .023))
        assert not gate._swinging.any()
    assert gate.load[[0, 3]].max() < 1.
    advance(gate, np.full(4, 60.), np.full(4, .023))
    assert not gate._swinging.any()  # Boundary sees last tick's still-partial load.
    advance(gate, np.full(4, 60.), np.full(4, .023))
    assert gate._swinging[[1, 2]].all()
    assert gate.time == pytest.approx(5.05)


@pytest.mark.parametrize('loads', [[.5]*4, [1., .5, 1., .5]])
def test_overlap_cannot_advance_without_an_existing_fully_loaded_diagonal(loads):
    gate = gait_at(5.01)
    gate.load[:] = loads
    gate.blocked = True
    advance(gate, np.full(4, 60.), np.full(4, .023))
    assert gate.time == pytest.approx(5.01)
    assert not gate._swinging.any()


def test_posture_and_current_force_checks_still_own_the_next_liftoff():
    gate = gait_at(5.01)
    for _ in range(3):
        advance(gate, np.full(4, 60.), np.full(4, .023))
    assert gate.time == pytest.approx(5.04)
    args = dict(dt=.01, gait_hz=2.2, duty=.6, horizon_dt=.02, horizon=12,
                settling=False, support_handover=True, foot_heights=np.full(4, .023))
    gate.update(np.full(4, 60.), liftoff_ready=False, **args)
    assert gate.time == pytest.approx(5.04)
    assert gate.posture_hold
    assert not gate._swinging.any()
    gate.update([0., 60., 60., 60.], liftoff_ready=True, **args)
    assert gate.time == pytest.approx(5.04)
    assert not gate._swinging.any()


def test_settling_and_cpu_keep_the_previous_overlap_clock_policy():
    for options in (dict(settling=True), dict(handover=False)):
        gate = gait_at(5.01)
        gate.blocked = True
        gate.load[[0, 3]] = .5
        advance(gate, np.full(4, 60.), np.full(4, .023), **options)
        assert gate.time == pytest.approx(5.01)


def test_nominal_four_foot_overlap_cannot_promise_a_still_unfinished_swing():
    gate = gait_at(5.01)
    gate._swinging[[0, 3]] = True
    gate.swing_elapsed[[0, 3]] = (1.-.6)/2.2-.02
    gate.confirmed[[0, 3]] = False
    gate.load[[0, 3]] = 0.
    schedule = advance(gate, [0., 60., 60., 0.], [.06, .023, .023, .06])
    assert gate.time == pytest.approx(5.02)
    assert gate._swinging[[0, 3]].all()
    assert gate.blocked
    np.testing.assert_array_equal(schedule[[0, 3]], np.zeros((2, 12)))
    np.testing.assert_array_equal(schedule[[1, 2]], np.ones((2, 12)))


@pytest.mark.parametrize('dt', [.01, .02, .06])
def test_a_tick_crossing_the_overlap_boundary_cannot_skip_current_support_checks(dt):
    gate = gait_at(5.04)
    gate.update([0., 60., 60., 60.], foot_heights=np.full(4, .023), dt=dt,
                gait_hz=2.2, duty=.6, horizon_dt=.02, horizon=12,
                settling=False, support_handover=True)
    assert gate.time == pytest.approx(5.04)
    assert not gate._swinging.any()
