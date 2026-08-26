from __future__ import annotations

import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.types import ALL_LEGS, LegId
from elesim_sim.robot.go2.mpc.contact_diagnostics import (
    GenesisContactDiagnostics,
    friction_ratio,
)


class _Tensor:
    """Small torch-like wrapper proving the existing conversion path is used."""

    def __init__(self, value: np.ndarray) -> None:
        self.value = np.asarray(value, dtype=float)

    def detach(self) -> "_Tensor":
        return self

    def cpu(self) -> "_Tensor":
        return self

    def numpy(self) -> np.ndarray:
        return self.value


class _Link:
    def __init__(self, index: int, *, velocity: tuple[float, float, float]) -> None:
        self.idx_local = index
        self._velocity = np.asarray(velocity, dtype=float)

    def get_pos(self) -> _Tensor:
        return _Tensor(np.array([0.0, 0.0, 1.0]))

    def get_quat(self) -> _Tensor:
        return _Tensor(np.array([1.0, 0.0, 0.0, 0.0]))

    def get_vel(self) -> _Tensor:
        return _Tensor(self._velocity)

    def get_ang(self) -> _Tensor:
        return _Tensor(np.array([0.0, 0.0, 2.0]))


class _Entity:
    def __init__(self) -> None:
        self.links = {
            f"{leg.value}_calf": _Link(index, velocity=(0.0, 0.0, 0.0))
            for index, leg in enumerate(ALL_LEGS)
        }
        self.contact_force_reads = 0

    def get_link(self, name: str) -> _Link:
        return self.links[name]

    def get_links_net_contact_force(self) -> _Tensor:
        self.contact_force_reads += 1
        return _Tensor(np.arange(12, dtype=float).reshape(4, 3))


def _offsets() -> dict[LegId, np.ndarray]:
    return {leg: np.array([0.0, 0.5, 0.0]) for leg in ALL_LEGS}


def _stance(value: bool = True) -> dict[LegId, bool]:
    return {leg: value for leg in ALL_LEGS}


def _grf(value: tuple[float, float, float] = (0.4, 0.0, 1.0)) -> dict[LegId, tuple[float, float, float]]:
    return {leg: value for leg in ALL_LEGS}


def test_foot_velocity_includes_angular_offset_and_reads_link_force() -> None:
    entity = _Entity()
    diagnostics = GenesisContactDiagnostics(
        entity,
        cadence_steps=1,
        foot_local_offsets=_offsets(),
    )

    sample = diagnostics.sample(
        step_index=0,
        elapsed_s=0.1,
        stance=_stance(),
        desired_grf_world=_grf(),
        physical_mu=0.5,
    )

    assert sample is not None
    fl = sample.by_leg()[LegId.FL]
    np.testing.assert_allclose(fl.position_world, [0.0, 0.5, 1.0])
    # omega=(0,0,2) crossed with offset=(0,0.5,0) gives (-1,0,0).
    np.testing.assert_allclose(fl.velocity_world, [-1.0, 0.0, 0.0])
    np.testing.assert_allclose(fl.net_contact_force_world, [0.0, 1.0, 2.0])
    assert fl.slip_speed_mps == pytest.approx(1.0)
    assert fl.slip_distance_m == pytest.approx(0.1)
    assert fl.friction_ratio == pytest.approx(0.8)
    assert entity.contact_force_reads == 1


def test_cadence_avoids_gpu_readback_until_due_and_accumulates_slip() -> None:
    entity = _Entity()
    diagnostics = GenesisContactDiagnostics(
        entity,
        cadence_steps=2,
        foot_local_offsets=_offsets(),
    )

    assert diagnostics.sample(
        step_index=1,
        elapsed_s=0.1,
        stance=_stance(),
        desired_grf_world=_grf(),
        physical_mu=0.5,
    ) is None
    assert entity.contact_force_reads == 0

    first = diagnostics.sample(
        step_index=2,
        elapsed_s=0.2,
        stance=_stance(),
        desired_grf_world=_grf(),
        physical_mu=0.5,
    )
    assert first is not None
    assert first.by_leg()[LegId.FL].slip_distance_m == pytest.approx(0.2)

    second = diagnostics.sample(
        step_index=4,
        elapsed_s=0.2,
        stance={**_stance(), LegId.FL: False},
        desired_grf_world=_grf(),
        physical_mu=0.5,
    )
    assert second is not None
    # Swing does not add slip, but the cumulative value remains observable.
    assert second.by_leg()[LegId.FL].slip_speed_mps == 0.0
    assert second.by_leg()[LegId.FL].slip_distance_m == pytest.approx(0.2)
    assert entity.contact_force_reads == 2


def test_force_probe_can_sample_off_cadence_and_swing_has_no_friction_ratio() -> None:
    entity = _Entity()
    diagnostics = GenesisContactDiagnostics(entity, cadence_steps=10)

    sample = diagnostics.sample(
        step_index=1,
        elapsed_s=0.0,
        stance=_stance(False),
        desired_grf_world=_grf(),
        physical_mu=0.8,
        force=True,
    )

    assert sample is not None
    assert all(foot.friction_ratio is None for foot in sample.feet)
    assert all(foot.slip_speed_mps == 0.0 for foot in sample.feet)


def test_invalid_physical_mu_and_incomplete_inputs_are_rejected() -> None:
    with pytest.raises(ValueError, match="physical_mu"):
        friction_ratio([0.0, 0.0, 1.0], physical_mu=0.0, stance=True)

    with pytest.raises(ValueError, match="cover FL/FR/RL/RR"):
        GenesisContactDiagnostics(_Entity(), foot_link_names={LegId.FL: "FL_calf"})

    diagnostics = GenesisContactDiagnostics(_Entity())
    with pytest.raises(ValueError, match="step_index"):
        diagnostics.sample(
            step_index=-1,
            elapsed_s=0.0,
            stance=_stance(),
            desired_grf_world=_grf(),
            physical_mu=0.8,
        )

