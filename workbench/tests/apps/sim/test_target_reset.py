"""Target reset restores physics state even when the cached position matches."""

import numpy as np
import pytest

from elesim_sim.runtime import SimScene


class Target:
    def __init__(self):
        self.calls = []

    def set_pos(self, pos):
        self.calls.append(("position", pos.copy()))

    def set_quat(self, quat):
        self.calls.append(("orientation", quat.copy()))

    def zero_all_dofs_velocity(self):
        self.calls.append(("velocity", None))


def test_reset_restores_cached_spawn_and_copies_position():
    target = Target()
    pos = np.array([0.5, 0.0, 0.4])
    scene = SimScene(sim_target_entity=target, sim_target_xyz=pos.copy())
    assert scene.reset_sim_target(pos)
    assert [name for name, _ in target.calls] == ["position", "orientation", "velocity"]
    np.testing.assert_array_equal(target.calls[1][1], [1.0, 0.0, 0.0, 0.0])
    pos[:] = 9
    np.testing.assert_array_equal(scene.sim_target_xyz, [0.5, 0.0, 0.4])


def test_reset_does_not_claim_success_if_orientation_fails():
    class BrokenTarget(Target):
        def set_quat(self, quat):
            raise RuntimeError("solver failed")

    previous = np.array([0.1, 0.2, 0.3])
    scene = SimScene(sim_target_entity=BrokenTarget(), sim_target_xyz=previous.copy())
    assert not scene.reset_sim_target(np.zeros(3))
    np.testing.assert_array_equal(scene.sim_target_xyz, previous)


def test_reset_rejects_nonfinite_position_before_writes():
    target = Target()
    scene = SimScene(sim_target_entity=target)
    with pytest.raises(ValueError, match="finite"):
        scene.reset_sim_target(np.array([np.nan, 0, 0]))
    assert target.calls == []


def test_reset_missing_target_reports_no_reset():
    assert not SimScene().reset_sim_target(np.zeros(3))
