"""A loaded diagonal may take over while the departing feet unload."""
import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_controller import ContactGatedGait, contact_schedule


def gait_at(time):
    gait = ContactGatedGait()
    gait.prime([50.]*4, dt=.04)
    gait.time = time
    planned = contact_schedule(time, gait_hz=2.2, duty=.6, dt=.02, horizon=12)[:, 0].astype(bool)
    gait._previous_planned = planned.copy()
    gait.confirmed &= planned
    gait.load *= planned
    return gait


def advance(gait, normal, heights, *, handover=True, settling=False):
    return gait.update(normal, foot_heights=heights, dt=.01, gait_hz=2.2,
                       duty=.6, horizon_dt=.02, horizon=12, settling=settling,
                       support_handover=handover)


@pytest.mark.parametrize('time,outgoing', [(5.01, [1, 2]), (.72, [0, 3])])
@pytest.mark.parametrize('already_blocked', [False, True])
def test_loaded_next_pair_advances_without_reconfirming_departing_pair(time, outgoing, already_blocked):
    gait = gait_at(time)
    gait.blocked = already_blocked
    normal, heights = np.full(4, 60.), np.full(4, .023)
    normal[outgoing] = 0.
    heights[outgoing] = .04
    schedule = advance(gait, normal, heights)
    assert gait.time == pytest.approx(time+.01)
    assert gait.phase_rate == 1.
    np.testing.assert_array_equal(schedule[outgoing, 0], 0.)
    np.testing.assert_array_equal(gait.confirmed[outgoing], False)


def test_overlap_progress_reaches_liftoff_with_only_the_verified_next_pair():
    gait = gait_at(5.01)
    gait.blocked = True
    gait.confirmed[[1, 2]] = False
    gait.load[[1, 2]] = 0.
    normal, heights = np.array([70., 0., 0., 70.]), np.array([.023, .04, .04, .023])
    for _ in range(5):
        schedule = advance(gait, normal, heights)
        np.testing.assert_array_equal(schedule[[1, 2], 0], 0.)
        np.testing.assert_array_equal(gait.load[[0, 3]], 1.)
    assert gait.time == pytest.approx(5.06)
    np.testing.assert_array_equal(gait._swinging, [False, True, True, False])


@pytest.mark.parametrize('incoming_load', [0., .5, 5/6])
def test_next_liftoff_waits_until_remaining_pair_is_fully_loaded(incoming_load):
    gait = gait_at(.72)
    gait.load[[1, 2]] = incoming_load
    gait.blocked = True
    advance(gait, np.full(4, 50.), np.full(4, .023))
    assert gait.time == pytest.approx(.72)
    assert gait.phase_rate == 0.
    assert not np.any(gait._swinging)


def test_incoming_load_ramp_overlaps_nominal_dwell_with_existing_loaded_support():
    gait = gait_at(5.01)
    gait.load[[0, 3]] = .5
    gait.blocked = True
    advance(gait, np.full(4, 50.), np.full(4, .023))
    assert gait.time == pytest.approx(5.02)
    assert gait.blocked
    assert not np.any(gait._swinging)


def test_overlap_does_not_advance_without_any_fully_loaded_diagonal():
    gait = gait_at(5.01)
    gait.load[:] = .5
    gait.blocked = True
    advance(gait, np.full(4, 50.), np.full(4, .023))
    assert gait.time == pytest.approx(5.01)


def test_new_airborne_evidence_blocks_liftoff_even_if_previous_load_was_full():
    gait = gait_at(.72)
    heights = np.full(4, .023)
    heights[1] = .04
    schedule = advance(gait, np.full(4, 50.), heights)
    assert gait.time == pytest.approx(.72)
    assert not gait.confirmed[1]
    assert schedule[1, 0] == 0.


@pytest.mark.parametrize('unloaded_force', [0., 2., 5.])
@pytest.mark.parametrize('time,remaining', [(.72, [1, 2]), (5.04, [0, 3])])
def test_new_liftoff_requires_present_ground_force_from_both_remaining_feet(
        unloaded_force, time, remaining):
    gait = gait_at(time)
    normal, heights = np.full(4, 60.), np.full(4, .023)
    normal[remaining[0]] = unloaded_force
    schedule = advance(gait, normal, heights)
    # A prior confirmation and completed ramp do not prove that the next
    # diagonal is still carrying the body when the other pair is lifted.
    assert gait.time == pytest.approx(time)
    assert not np.any(gait._swinging)
    np.testing.assert_array_equal(schedule[:, 0], np.ones(4))
    assert gait.confirmed[remaining[0]]
    assert gait.load[remaining[0]] == 1.
    normal[remaining[0]] = 60.
    advance(gait, normal, heights)
    assert gait.time == pytest.approx(time + .01)
    assert np.any(gait._swinging)


def test_brief_force_gap_during_stance_does_not_freeze_an_existing_swing():
    gait = gait_at(.35)
    advance(gait, [50., 0., 50., 50.], np.full(4, .023))
    assert gait.time == pytest.approx(.36)
    assert gait.confirmed[1]
    assert gait.load[1] == 1.


def test_single_support_cannot_advance_after_losing_a_required_foot():
    gait = gait_at(.35)
    heights = np.full(4, .023)
    heights[1] = .04
    advance(gait, np.full(4, 50.), heights)
    assert gait.time == pytest.approx(.35)


def test_settling_and_legacy_gate_keep_the_clock_frozen():
    for options in [dict(settling=True), dict(handover=False)]:
        gait = gait_at(5.01)
        gait.blocked = True
        advance(gait, np.full(4, 50.), np.full(4, .023), **options)
        assert gait.time == pytest.approx(5.01)


def test_departing_search_foot_starts_a_fresh_swing_reference():
    from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
    c, state = controller()
    state.contacts[:] = 0.
    state.footholds_world[:] = [2., .2, .025]
    c._last_contacts[:] = 0.
    c._was_searching[:] = True
    c._contact_gait.search[:] = False
    c._contact_gait.swing_progress[:] = 0.
    feet = np.tile([1., .2, .033], (4, 1))
    c._swing_starts[:] = -9.
    c._touchdowns[:] = -8.
    c._torques(state, feet, np.zeros((4, 3)), np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    np.testing.assert_array_equal(c._swing_starts, feet)
    np.testing.assert_array_equal(c._touchdowns, state.footholds_world)


@pytest.mark.parametrize('backend,restarted', [('jax_mppi', True), ('acados', False)])
def test_foothold_selection_replans_only_search_to_swing_and_preserves_contact_history(backend, restarted):
    from elesim_sim.robot.go2.pympc_controller import planned_footholds
    from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
    c, state = controller()
    c._solver.backend = backend
    c._last_contacts[:] = 0.
    c._was_searching[:] = [True, True, False, False]
    c._contact_gait.search[:] = [False, True, False, False]
    contacts = np.array([0., 0., 1., 0.])
    previous = c._swing_previous_contacts(contacts)
    np.testing.assert_array_equal(previous, [float(restarted), 0., 0., 0.])
    np.testing.assert_array_equal(c._last_contacts, np.zeros(4))
    feet = np.arange(12).reshape(4, 3)
    candidates, latched = feet+10., feet+20.
    chosen = planned_footholds(feet, candidates, contacts, previous, latched)
    np.testing.assert_array_equal(chosen[0], candidates[0] if restarted else latched[0])
    np.testing.assert_array_equal(chosen[1], latched[1])
    np.testing.assert_array_equal(chosen[2], feet[2])
    np.testing.assert_array_equal(chosen[3], latched[3])
