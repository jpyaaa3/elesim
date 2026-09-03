"""Low-cadence, read-only contact diagnostics for the Genesis Go2 model.

The diagnostics deliberately live outside the MPC control path.  Genesis tensors
are copied to NumPy only when :meth:`GenesisContactDiagnostics.sample` is due,
which keeps GPU readbacks out of the simulation step while making the actual
plant/contact state observable in tests and experiments.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from elesim_sim.robot.go2.locomotion.types import ALL_LEGS, LegId
from elesim_sim.simulation.genesis.utils import to_numpy_1d


DEFAULT_FOOT_LINK_NAMES: Mapping[LegId, str] = {
    leg: f"{leg.value}_calf" for leg in ALL_LEGS
}
"""Genesis Go2 uses the calf link as the merged foot link."""

DEFAULT_FOOT_LOCAL_OFFSETS: Mapping[LegId, np.ndarray] = {
    leg: np.array([0.0, 0.0, -0.213], dtype=float) for leg in ALL_LEGS
}
"""Foot-tip offsets in each calf link's local frame (metres)."""


@dataclass(frozen=True)
class FootContactDiagnostic:
    """One leg's kinematics and contact metrics at a diagnostics sample."""

    leg: LegId
    position_world: np.ndarray
    velocity_world: np.ndarray
    net_contact_force_world: np.ndarray
    stance: bool
    desired_grf_world: np.ndarray
    slip_speed_mps: float
    slip_distance_m: float
    friction_ratio: float | None


@dataclass(frozen=True)
class ContactDiagnosticsSample:
    """A coherent low-cadence sample for all four Go2 legs."""

    step_index: int
    elapsed_s: float
    feet: tuple[FootContactDiagnostic, ...]

    def by_leg(self) -> dict[LegId, FootContactDiagnostic]:
        return {foot.leg: foot for foot in self.feet}


def _vector3(raw: object, *, name: str) -> np.ndarray:
    value = to_numpy_1d(raw)
    if value.size != 3:
        raise ValueError(f"{name} must contain exactly 3 values, got {value.size}")
    return value.astype(float, copy=True)


def _quaternion4(raw: object, *, name: str) -> np.ndarray:
    value = to_numpy_1d(raw)
    if value.size != 4:
        raise ValueError(f"{name} must contain exactly 4 values, got {value.size}")
    return value.astype(float, copy=True)


def _rotate_wxyz(quaternion_wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate one vector by a Genesis ``wxyz`` quaternion without readback deps."""

    w, x, y, z = (float(value) for value in quaternion_wxyz)
    norm = float(np.linalg.norm(quaternion_wxyz))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("link quaternion must have a finite, non-zero norm")
    w, x, y, z = (value / norm for value in (w, x, y, z))
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return rotation @ vector


def _leg_key(leg: LegId | str) -> LegId:
    if isinstance(leg, LegId):
        return leg
    try:
        return LegId(str(leg))
    except ValueError as exc:
        raise ValueError(f"unknown Go2 leg {leg!r}") from exc


def _normalise_leg_map(
    values: Mapping[LegId | str, object], *, name: str
) -> dict[LegId, object]:
    normalised = {_leg_key(leg): value for leg, value in values.items()}
    missing = [leg.value for leg in ALL_LEGS if leg not in normalised]
    extra = [str(leg) for leg in normalised if leg not in ALL_LEGS]
    if missing or extra:
        raise ValueError(f"{name} must cover FL/FR/RL/RR; missing={missing}, extra={extra}")
    return normalised


def _read_foot_pose_velocity(link: object, offset_local: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Read a calf link and transform its fixed foot offset into world space."""

    position_origin = _vector3(link.get_pos(), name="link position")
    quaternion_wxyz = _quaternion4(link.get_quat(), name="link quaternion")
    velocity_origin = _vector3(link.get_vel(), name="link linear velocity")
    angular_velocity_world = _vector3(link.get_ang(), name="link angular velocity")

    offset_world = _rotate_wxyz(quaternion_wxyz, offset_local)
    position_world = position_origin + offset_world
    velocity_world = velocity_origin + np.cross(angular_velocity_world, offset_world)
    return position_world, velocity_world


def _read_net_contact_forces(entity: object, links: Sequence[object]) -> dict[LegId, np.ndarray]:
    """Read one link-indexed net-contact-force tensor for the four calf links."""

    if not hasattr(entity, "get_links_net_contact_force"):
        raise AttributeError("Genesis entity lacks get_links_net_contact_force()")
    raw = to_numpy_1d(entity.get_links_net_contact_force())
    if raw.size == 0 or raw.size % 3:
        raise ValueError(
            "get_links_net_contact_force() must return a non-empty array of 3-vectors"
        )
    rows = raw.reshape(-1, 3)
    forces: dict[LegId, np.ndarray] = {}
    for leg, link in zip(ALL_LEGS, links):
        if not hasattr(link, "idx_local"):
            raise AttributeError(f"{leg.value} calf link lacks idx_local")
        index = int(link.idx_local)
        if index < 0 or index >= len(rows):
            raise IndexError(f"{leg.value} calf link index {index} outside contact-force tensor")
        forces[leg] = rows[index].astype(float, copy=True)
    return forces


def friction_ratio(
    desired_grf_world: Sequence[float] | np.ndarray,
    *,
    physical_mu: float,
    stance: bool,
) -> float | None:
    """Return desired tangential force / physical Coulomb limit.

    A swing leg or a non-positive desired normal force has no meaningful
    friction utilization, so it returns ``None``.  A non-positive or non-finite
    physical coefficient is a configuration error and raises ``ValueError``.
    """

    mu = float(physical_mu)
    if not np.isfinite(mu) or mu <= 0.0:
        raise ValueError(f"physical_mu must be finite and positive, got {physical_mu!r}")
    force = _vector3(desired_grf_world, name="desired GRF")
    normal = float(force[2])
    if not stance or normal <= 0.0:
        return None
    return float(np.linalg.norm(force[:2]) / (mu * normal))


class GenesisContactDiagnostics:
    """Low-cadence Genesis reader plus pure slip/contact metrics."""

    def __init__(
        self,
        entity: object,
        *,
        cadence_steps: int = 5,
        foot_link_names: Mapping[LegId | str, str] | None = None,
        foot_local_offsets: Mapping[LegId | str, Sequence[float]] | None = None,
    ) -> None:
        cadence = int(cadence_steps)
        if cadence < 1:
            raise ValueError(f"cadence_steps must be >= 1, got {cadence_steps!r}")
        self._entity = entity
        self._cadence_steps = cadence
        names = _normalise_leg_map(
            foot_link_names or DEFAULT_FOOT_LINK_NAMES,
            name="foot_link_names",
        )
        offsets = _normalise_leg_map(
            foot_local_offsets or DEFAULT_FOOT_LOCAL_OFFSETS,
            name="foot_local_offsets",
        )
        self._foot_link_names = {leg: str(value) for leg, value in names.items()}
        self._foot_local_offsets = {
            leg: _vector3(value, name=f"{leg.value} foot offset")
            for leg, value in offsets.items()
        }
        self._slip_distance_m = {leg: 0.0 for leg in ALL_LEGS}

    @property
    def cadence_steps(self) -> int:
        return self._cadence_steps

    def reset(self) -> None:
        """Reset cumulative stance slip without touching the Genesis entity."""

        self._slip_distance_m = {leg: 0.0 for leg in ALL_LEGS}

    def should_sample(self, step_index: int) -> bool:
        index = int(step_index)
        if index < 0:
            raise ValueError(f"step_index must be non-negative, got {step_index!r}")
        return index % self._cadence_steps == 0

    def sample(
        self,
        *,
        step_index: int,
        elapsed_s: float,
        stance: Mapping[LegId | str, bool],
        desired_grf_world: Mapping[LegId | str, Sequence[float] | np.ndarray],
        physical_mu: float,
        force: bool = False,
    ) -> ContactDiagnosticsSample | None:
        """Read one sample when cadence is due; ``force`` is for explicit probes."""

        index = int(step_index)
        elapsed = float(elapsed_s)
        if index < 0:
            raise ValueError(f"step_index must be non-negative, got {step_index!r}")
        if not np.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError(f"elapsed_s must be finite and non-negative, got {elapsed_s!r}")
        if not force and not self.should_sample(index):
            return None

        stance_by_leg = _normalise_leg_map(stance, name="stance")
        grf_by_leg = _normalise_leg_map(desired_grf_world, name="desired_grf_world")
        links = [self._entity.get_link(self._foot_link_names[leg]) for leg in ALL_LEGS]
        forces = _read_net_contact_forces(self._entity, links)

        feet: list[FootContactDiagnostic] = []
        for leg, link in zip(ALL_LEGS, links):
            is_stance = bool(stance_by_leg[leg])
            desired = _vector3(grf_by_leg[leg], name=f"{leg.value} desired GRF")
            position, velocity = _read_foot_pose_velocity(
                link,
                self._foot_local_offsets[leg],
            )
            slip_speed = float(np.linalg.norm(velocity[:2])) if is_stance else 0.0
            if is_stance:
                self._slip_distance_m[leg] += slip_speed * elapsed
            feet.append(
                FootContactDiagnostic(
                    leg=leg,
                    position_world=position,
                    velocity_world=velocity,
                    net_contact_force_world=forces[leg],
                    stance=is_stance,
                    desired_grf_world=desired,
                    slip_speed_mps=slip_speed,
                    slip_distance_m=float(self._slip_distance_m[leg]),
                    friction_ratio=friction_ratio(
                        desired,
                        physical_mu=physical_mu,
                        stance=is_stance,
                    ),
                )
            )
        return ContactDiagnosticsSample(index, elapsed, tuple(feet))
