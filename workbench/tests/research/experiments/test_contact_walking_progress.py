from workbench.research.debug.profile_sim_walking import has_commanded_progress
from dataclasses import dataclass

import pytest


def test_stationary_or_opposite_motion_cannot_pass_a_walking_trial():
    assert not has_commanded_progress((.8, 0, 0), [0, 0, 0], duration=6)
    assert not has_commanded_progress((.8, 0, 0), [-1, 0, 0], duration=6)
    assert not has_commanded_progress((0, -.35, 0), [1, 0, 0], duration=6)


def test_forward_back_left_right_need_progress_along_command():
    for command in [(0.8, 0, 0), (-0.8, 0, 0), (0, .35, 0), (0, -.35, 0)]:
        assert has_commanded_progress(command, [command[0], command[1], 0], duration=6)


def test_cadence_probe_changes_feedback_without_changing_integration_interval():
    from workbench.research.debug.profile_sim_walking import override_physics
    @dataclass(frozen=True)
    class Physics:
        dt: float = .02
        substeps: int = 4
    original = Physics()
    faster = override_physics(original, dt=.005, substeps=1)
    assert faster.dt/faster.substeps == original.dt/original.substeps
    assert original == Physics()
    assert override_physics(original) == original


@pytest.mark.parametrize('options', [{'dt':0.}, {'dt':float('nan')},
                                     {'substeps':0}, {'substeps':1.5}])
def test_cadence_probe_rejects_invalid_physics_without_scientific_imports(options):
    from workbench.research.debug.profile_sim_walking import override_physics
    @dataclass(frozen=True)
    class Physics:
        dt: float = .02
        substeps: int = 4
    with pytest.raises(ValueError):
        override_physics(Physics(), **options)


def stop_samples():
    return [dict(t=t, pose_stage='stand', faulted=False, fallen=False)
            for t in (9.98, 17.98, 25.98)]


def summarize_stops(samples, **changes):
    from workbench.research.debug.profile_sim_walking import stop_cycle_results
    options = dict(settle=2., duration=6., stop=2., cycles=3, sample_interval=.02)
    options.update(changes)
    return stop_cycle_results(samples, **options)


def test_walking_probe_checks_each_stop_before_the_next_restart():
    rows = summarize_stops(stop_samples())
    assert [row['cycle'] for row in rows] == [1, 2, 3]
    assert all(row['completed'] for row in rows)


@pytest.mark.parametrize('change', [dict(pose_stage='walk'), dict(faulted=True), dict(fallen=True)])
def test_final_stand_does_not_hide_a_failed_intermediate_stop(change):
    samples = stop_samples()
    samples[1].update(change)
    rows = summarize_stops(samples)
    assert [row['completed'] for row in rows] == [True, False, True]
    assert rows[1]['last_t'] == 17.98


def test_missing_or_truncated_stop_cannot_pass():
    assert not any(row['completed'] for row in summarize_stops([]))
    samples = stop_samples()
    samples[1]['t'] = 16.5
    assert not summarize_stops(samples)[1]['completed']
    assert not summarize_stops(stop_samples()[:1])[1]['completed']


def test_stop_check_uses_the_actual_diagnostic_sample_interval():
    samples = stop_samples()
    samples[0]['t'] = 9.95
    assert not summarize_stops(samples)[0]['completed']
    assert summarize_stops(samples, sample_interval=.1)[0]['completed']
    assert summarize_stops(samples, stop=0.) == []
