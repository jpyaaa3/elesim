import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_controller import ContactGatedGait, swing_foot_reference


def tick(gate, force=(50., 50., 50., 50.), heights=(.02, .02, .02, .02)):
    return gate.update(force, dt=.02, gait_hz=2.5, duty=.6,
                       horizon_dt=.02, horizon=12, settling=False, foot_heights=heights)


def ready():
    gate = ContactGatedGait()
    for _ in range(2):
        gate.prime([50.] * 4, dt=.02)
    return gate


def test_existing_support_is_not_unloaded_at_start():
    gate = ready()
    np.testing.assert_array_equal(tick(gate)[:, 0], np.ones(4))
    np.testing.assert_array_equal(gate.load, np.ones(4))
    assert not gate.search.any()


def test_brief_unloading_preserves_support_and_load():
    gate = ready()
    tick(gate, [0., 50., 50., 50.])
    assert gate.confirmed[0] and gate.load[0] == 1.
    tick(gate)
    assert gate.load[0] == 1.


@pytest.mark.parametrize('clearance_height', [.032, .04, .08])
@pytest.mark.parametrize('normal', [0., 50.])
def test_airborne_support_loss_prevents_next_pair_liftoff_in_same_tick(clearance_height, normal):
    gate = ready()
    gate.time = .02  # FR/RL liftoff is reached on the next tick.
    schedule = tick(gate, [normal, 50., 50., 50.], [clearance_height, .02, .02, .02])
    assert gate.time == pytest.approx(.02)
    np.testing.assert_array_equal(schedule[:, 0], [0., 1., 1., 1.])
    assert gate.search[0]


def test_airborne_impulses_do_not_confirm_ground_landing():
    gate = ContactGatedGait()
    # The authored foot sphere is 22 mm radius; even allowing its 2 mm
    # origin offset, a 34 mm foot center is clear of the flat ground.
    # Cached/impulsive force alone must not override that geometry.
    tick(gate, heights=[.034]*4)
    tick(gate, heights=[.034]*4)
    assert not gate.confirmed.any()
    assert not gate.load.any()
    tick(gate)  # Airborne impulses must not count toward ground evidence.
    assert not gate.confirmed.any()
    tick(gate)
    assert gate.confirmed.all()


def test_airborne_loss_requires_fresh_ground_evidence_before_reloading():
    gate = ContactGatedGait()
    tick(gate)
    tick(gate)
    assert gate.confirmed.all()
    tick(gate, heights=[.034, .02, .02, .02])
    assert not gate.confirmed[0]
    tick(gate)
    assert not gate.confirmed[0]
    tick(gate)
    assert gate.confirmed[0]


def test_missing_contact_never_becomes_support_and_times_out():
    gate = ContactGatedGait()
    for _ in range(14):
        assert not tick(gate, [0.] * 4, [.08] * 4).any()
    with pytest.raises(RuntimeError, match="touchdown timeout"):
        for _ in range(3):
            tick(gate, [0.] * 4, [.08] * 4)


def test_one_impulse_does_not_confirm_landing():
    gate = ContactGatedGait()
    tick(gate)
    assert not gate.confirmed.any()
    tick(gate, [0.] * 4)
    assert not gate.confirmed.any()


def test_repeated_ground_contact_with_one_unloaded_tick_confirms_landing():
    gate = ContactGatedGait()
    tick(gate)
    tick(gate, [0.] * 4)
    tick(gate)
    assert gate.confirmed.all()
    assert not gate.search.any()


def test_landing_load_ramps_and_releases_gait_clock():
    gate = ContactGatedGait()
    tick(gate)
    tick(gate)
    assert np.all((gate.load > 0) & (gate.load < 1))
    assert gate.blocked
    tick(gate)
    tick(gate)
    assert not gate.blocked
    assert gate.time == 0.
    tick(gate)
    assert gate.time > 0.


def test_liftoff_clears_previous_contact_and_load():
    gate = ready()
    gate.time = .06
    schedule = tick(gate)
    np.testing.assert_array_equal(schedule[:, 0], [1., 0., 0., 1.])
    assert not gate.confirmed[[1, 2]].any()
    assert not gate.load[[1, 2]].any()


def test_already_airborne_feet_finish_swing_while_other_support_is_missing():
    gate = ready()
    gate.time = .04
    tick(gate)  # FR/RL have started their swing.
    for _ in range(10):
        schedule = tick(gate, [0., 50., 50., 50.], [.08, .02, .02, .02])
    assert gate.search[0]
    np.testing.assert_array_equal(schedule[[1, 2], 0], [1., 1.])


def test_foothold_stance_forecast_includes_measured_support_handover_delays():
    gate = ready()
    time_s = 0.
    previous_swing = np.zeros(4, dtype=bool)
    liftoffs = []
    for _ in range(300):
        grounded = ~gate._swinging
        gate.update(np.where(grounded, 50., 0.), dt=.01, gait_hz=2.5, duty=.6,
                    horizon_dt=.02, horizon=12, settling=False,
                    foot_heights=np.where(grounded, .022, .07), support_handover=True)
        time_s += .01
        if gate._swinging[0] and not previous_swing[0]:
            liftoffs.append(time_s)
        previous_swing = gate._swinging.copy()
    measured_cycle = np.median(np.diff(liftoffs[-4:]))
    assert measured_cycle > .4+.1
    assert gate.stance_duration_s(gait_hz=2.5, duty=.6) == pytest.approx(measured_cycle-.16)


def test_stance_forecast_starts_nominal_and_forgets_old_cycles_during_settling():
    gate = ready()
    assert gate.stance_duration_s(gait_hz=2.5, duty=.6) == pytest.approx(.24)
    for _ in range(180):
        grounded = ~gate._swinging
        gate.update(np.where(grounded, 50., 0.), dt=.01, gait_hz=2.5, duty=.6,
                    horizon_dt=.02, horizon=12, settling=False,
                    foot_heights=np.where(grounded, .022, .07), support_handover=True)
    assert gate.stance_duration_s(gait_hz=2.5, duty=.6) > .34
    gate.update([50.]*4, dt=.01, gait_hz=2.5, duty=.6, horizon_dt=.02,
                horizon=12, settling=True, foot_heights=[.022]*4, support_handover=True)
    assert gate.stance_duration_s(gait_hz=2.5, duty=.6) == pytest.approx(.24)


@pytest.mark.parametrize("progress", [0., 1.])
def test_swing_has_zero_endpoint_velocity(progress):
    _, velocity = swing_foot_reference(progress, np.array([0., 0., .025]),
                                      np.array([.15, .02, .025]), duration_s=.16)
    np.testing.assert_allclose(velocity, np.zeros(3), atol=1e-10)


def test_paused_gait_holds_swing_position_without_velocity_feedforward():
    start, end = np.array([0., .1, .02]), np.array([.1, .12, .025])
    position, velocity = swing_foot_reference(.4, start, end, duration_s=.16, phase_rate=0.)
    np.testing.assert_array_equal(position, swing_foot_reference(.4, start, end, duration_s=.16)[0])
    np.testing.assert_array_equal(velocity, np.zeros(3))
