"""One diverged environment must not end the run.

Genesis raises for the whole scene when the constraint solver produces nan --
"Invalid constraint forces causing 'nan'" -- and a 600 iteration run died at
1266 on one such step, with no checkpoint newer than 16 iterations back.  The
errno behind it is per-env and writable, so the envs that actually diverged can
be identified, reset, and the batch carried on.

What is pinned here: which envs get reset, that the errno is cleared so the
next step is not refused as well, that the affected episodes end, and that an
error the solver cannot attribute to any env still propagates.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import pytest
import torch

from elesim_sim.rl.envs.wrap_env import WrapGraspEnv


class _Errno:
    """Stands in for the taichi array behind `rigid_solver._errno`."""

    def __init__(self, values: torch.Tensor) -> None:
        self.values = values


class _Solver:
    def __init__(self, mask: torch.Tensor | None) -> None:
        self._mask = mask
        self._errno = _Errno(
            torch.zeros(0) if mask is None else mask.to(torch.int32).clone()
        )

    def get_error_envs_mask(self):
        if self._mask is None:
            raise AttributeError("no mask")
        return self._mask


class _Sim:
    def __init__(self, solver) -> None:
        self.rigid_solver = solver


class _GenesisScene:
    def __init__(self, solver) -> None:
        self.sim = _Sim(solver)


class _Scene:
    """The slice of WrapGraspScene the recovery path uses."""

    def __init__(self, solver, *, raises: int) -> None:
        self.scene = _GenesisScene(solver)
        self._raises = raises
        self.steps = 0

    def step(self):
        self.steps += 1
        if self._raises > 0:
            self._raises -= 1
            raise RuntimeError("Invalid constraint forces causing 'nan'.")


class _Env(WrapGraspEnv):
    """Only the recovery path, with no Genesis scene behind it."""

    def __init__(self, n, solver, *, raises):
        self.num_envs = n
        self.device = torch.device("cpu")
        self.scene = _Scene(solver, raises=raises)
        self._diverged = torch.zeros(n, dtype=torch.bool)
        self.reset_calls: list[list[int]] = []
        self.zeroed: list[list[int]] = []

    def _reset_idx(self, env_ids):
        self.reset_calls.append(sorted(int(v) for v in env_ids))

    def _zero_object_velocity(self, env_ids):
        self.zeroed.append(sorted(int(v) for v in env_ids))


def _patch_qd(monkeypatch, holder):
    """Route genesis's qd_to_torch at the module the recovery imports it from."""
    import sys
    import types

    mod = types.ModuleType("genesis.utils.misc")
    mod.qd_to_torch = lambda arr: arr.values
    pkg = types.ModuleType("genesis.utils")
    pkg.misc = mod
    root = sys.modules.get("genesis") or types.ModuleType("genesis")
    monkeypatch.setitem(sys.modules, "genesis", root)
    monkeypatch.setitem(sys.modules, "genesis.utils", pkg)
    monkeypatch.setitem(sys.modules, "genesis.utils.misc", mod)
    return holder


def test_only_the_diverged_envs_are_reset(monkeypatch):
    mask = torch.tensor([False, True, False, True])
    solver = _Solver(mask)
    _patch_qd(monkeypatch, solver)
    env = _Env(4, solver, raises=1)
    env._step_scene_or_recover()
    assert env.reset_calls == [[1, 3]]
    assert env.zeroed == [[1, 3]]


def test_the_diverged_envs_are_flagged(monkeypatch):
    mask = torch.tensor([False, True, False, True])
    solver = _Solver(mask)
    _patch_qd(monkeypatch, solver)
    env = _Env(4, solver, raises=1)
    env._step_scene_or_recover()
    assert env._diverged.tolist() == [False, True, False, True]


def test_the_errno_is_cleared_so_the_next_step_is_not_refused(monkeypatch):
    mask = torch.tensor([True, False])
    solver = _Solver(mask)
    _patch_qd(monkeypatch, solver)
    env = _Env(2, solver, raises=1)
    env._step_scene_or_recover()
    assert int(solver._errno.values.sum()) == 0


def test_a_clean_step_touches_nothing(monkeypatch):
    solver = _Solver(torch.tensor([False, False]))
    _patch_qd(monkeypatch, solver)
    env = _Env(2, solver, raises=0)
    env._step_scene_or_recover()
    assert env.reset_calls == []
    assert env._diverged.tolist() == [False, False]


def test_an_unattributable_error_still_propagates(monkeypatch):
    # No env flagged: the solver cannot say which one, so swallowing it would
    # hide a fault that belongs to the whole scene.
    solver = _Solver(torch.tensor([False, False]))
    _patch_qd(monkeypatch, solver)
    env = _Env(2, solver, raises=1)
    with pytest.raises(RuntimeError, match="nan"):
        env._step_scene_or_recover()


def test_a_missing_solver_still_propagates(monkeypatch):
    solver = _Solver(None)
    _patch_qd(monkeypatch, solver)
    env = _Env(2, solver, raises=1)
    with pytest.raises(RuntimeError, match="nan"):
        env._step_scene_or_recover()


# ---------------------------------------------------------------------------
# The step after a recovery has to hand back a finite observation
#
# rsl_rl checks the observation before it looks at `dones`, so terminating the
# episode is not enough: a run died with "observation group 'privileged'
# contains NaN" one step after a recovery, because values carried through that
# step -- contact accumulations, displacement, tilt -- were still nan.
# ---------------------------------------------------------------------------

class _ObsEnv(WrapGraspEnv):
    def __init__(self, n, obs):
        self.num_envs = n
        self.device = torch.device("cpu")
        self._obs = obs


def _obs(n, width=4):
    return {
        "policy": torch.zeros(n, width),
        "privileged": torch.zeros(n, width),
    }


def test_nan_in_a_recovered_env_is_cleared():
    obs = _obs(3)
    obs["privileged"][1, 2] = float("nan")
    env = _ObsEnv(3, obs)
    env._sanitise_recovered_observations(torch.tensor([False, True, False]))
    assert torch.isfinite(obs["privileged"]).all()


def test_infinities_are_cleared_too():
    obs = _obs(2)
    obs["policy"][0, 0] = float("inf")
    obs["privileged"][0, 1] = float("-inf")
    env = _ObsEnv(2, obs)
    env._sanitise_recovered_observations(torch.tensor([True, False]))
    assert torch.isfinite(obs["policy"]).all()
    assert torch.isfinite(obs["privileged"]).all()


def test_a_nan_outside_the_recovered_envs_is_left_alone():
    # A different fault, and it should still reach rsl_rl rather than be
    # quietly zeroed.
    obs = _obs(3)
    obs["privileged"][2, 0] = float("nan")
    env = _ObsEnv(3, obs)
    env._sanitise_recovered_observations(torch.tensor([True, False, False]))
    assert not torch.isfinite(obs["privileged"][2, 0])


def test_healthy_values_in_a_recovered_env_survive():
    obs = _obs(2)
    obs["policy"][0] = torch.tensor([1.0, 2.0, 3.0, 4.0])
    env = _ObsEnv(2, obs)
    env._sanitise_recovered_observations(torch.tensor([True, False]))
    assert obs["policy"][0].tolist() == [1.0, 2.0, 3.0, 4.0]


def test_nothing_recovered_is_a_no_op():
    obs = _obs(2)
    obs["privileged"][0, 0] = float("nan")
    env = _ObsEnv(2, obs)
    env._sanitise_recovered_observations(torch.tensor([False, False]))
    assert not torch.isfinite(obs["privileged"][0, 0])
