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
