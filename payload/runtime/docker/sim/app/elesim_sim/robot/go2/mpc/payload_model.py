from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from elesim_sim.simulation.genesis.utils import quat_wxyz_to_xyzw as _quat_wxyz_to_xyzw, to_numpy_1d as _to_numpy_1d


def inertia_in_link_frame(description) -> np.ndarray | None:
    """Genesis stores inertia in its principal frame, separately from the link."""
    if description.inertia is None:
        return None
    tensor = np.asarray(description.inertia, dtype=float).reshape(3, 3)
    quaternion = getattr(description, "inertial_quat", None)
    if quaternion is None:
        return tensor
    rotation = Rot.from_quat(_quat_wxyz_to_xyzw(quaternion)).as_matrix()
    return rotation @ tensor @ rotation.T


def _shift_inertia(
    inertia: np.ndarray,
    mass: float,
    from_com: np.ndarray,
    to_com: np.ndarray,
) -> np.ndarray:
    s = np.asarray(from_com, dtype=float).reshape(3) - np.asarray(to_com, dtype=float).reshape(3)
    return np.asarray(inertia, dtype=float).reshape(3, 3) + mass * (
        float(np.dot(s, s)) * np.eye(3) - np.outer(s, s)
    )


@dataclass(frozen=True)
class ArmPayloadSnapshot:
    mass_kg: float
    com_world: np.ndarray
    vel_world: np.ndarray
    inertia_world: np.ndarray


@dataclass(frozen=True)
class _ArmPayloadProperties:
    links: tuple
    masses: np.ndarray
    measured_mass: float
    offsets: np.ndarray
    inertias: np.ndarray


class ArmPayloadCompensator:
    """Estimate welded arm inertial properties from Genesis and patch PinGo2Model COM state."""

    def __init__(
        self,
        arm_entity,
        *,
        mass_override_kg: float = 0.0,
        link_names: set[str] | None = None,
    ) -> None:
        self._arm = arm_entity
        self._mass_override = max(0.0, float(mass_override_kg))
        names = {str(name).strip() for name in (link_names or set()) if str(name).strip()}
        self._link_names = names if names else None
        self._link_ids: set[int] = set()
        self._previous_com_world: np.ndarray | None = None
        self._warned_stale_velocity = False
        self._properties: _ArmPayloadProperties | None = None
        self._properties_ready = False
        if self._link_names is not None:
            missing: list[str] = []
            for name in self._link_names:
                try:
                    self._link_ids.add(id(arm_entity.get_link(name)))
                except (KeyError, ValueError, AttributeError):
                    missing.append(name)
            if missing:
                raise ValueError(
                    f"Genesis arm entity is missing configured payload links: {sorted(missing)}"
                )

    @staticmethod
    def _link_name(link) -> str:
        for attr in ("name", "_name"):
            value = getattr(link, attr, None)
            if value is not None:
                return str(value).strip()
        return ""

    def reset(self) -> None:
        self._previous_com_world = None
        self._warned_stale_velocity = False
        self._properties = None
        self._properties_ready = False

    def rebind_entity(self, entity) -> None:
        self._arm = entity
        self._link_ids = ({id(entity.get_link(name)) for name in self._link_names}
                          if self._link_names is not None else set())
        self.reset()

    def _prepare_properties(self) -> _ArmPayloadProperties | None:
        if self._properties_ready:
            return self._properties
        links, masses, offsets, inertias = [], [], [], []
        for link in self._arm.links:
            if (
                self._link_names is not None
                and self._link_name(link) not in self._link_names
                and id(link) not in self._link_ids
            ):
                continue
            description = link.desc
            if description.mass is None or float(description.mass) <= 1e-9:
                continue
            links.append(link)
            masses.append(float(description.mass))
            offsets.append(np.zeros(3) if description.inertial_pos is None else
                           np.asarray(description.inertial_pos, dtype=float).reshape(3))
            inertia = inertia_in_link_frame(description)
            inertias.append(np.zeros((3, 3)) if inertia is None else inertia)
        if links:
            self._properties = _ArmPayloadProperties(
                tuple(links), np.asarray(masses), float(sum(masses)),
                np.stack(offsets), np.stack(inertias),
            )
        # Genesis link topology and authored inertial properties are fixed
        # between scene resets. World pose and velocity remain live each tick.
        self._properties_ready = True
        return self._properties

    def _link_states(self, links, *, snapshot=None):
        fields = (('pos', 3), ('quat', 4), ('vel', 3), ('ang', 3))
        if snapshot is not None and snapshot.entity is self._arm:
            indices = [link.idx_local for link in links]
            values = [getattr(snapshot, name)[indices] for name, _ in fields]
            for (name, _), value in zip(fields, values):
                if not np.all(np.isfinite(value)):
                    raise ValueError(f'payload {name} must contain finite vectors')
            return tuple(values)
        methods = [getattr(self._arm, f'get_links_{name}', None) for name, _ in fields]
        if not all(callable(method) for method in methods) or not all(
                hasattr(link, 'idx_local') for link in links):
            rows = [tuple(_to_numpy_1d(getattr(link, f'get_{name}')())[:width]
                          for name, width in fields) for link in links]
            return tuple(np.stack(column) for column in zip(*rows))
        # Entity and link getters use the same authored link origin and world
        # velocity. Read each field once instead of synchronizing four times
        # per arm link on every control tick.
        indices = np.array([link.idx_local for link in links], dtype=np.int32)
        values = []
        for (name, width), method in zip(fields, methods):
            flat = _to_numpy_1d(method(links_idx_local=indices))
            if flat.size != len(links)*width or not np.all(np.isfinite(flat)):
                raise ValueError(f'payload {name} must contain {len(links)} finite {width}-vectors')
            values.append(flat.reshape(len(links), width))
        return tuple(values)

    def measure(self, *, dt: float | None = None, snapshot=None) -> ArmPayloadSnapshot | None:
        elapsed = None if dt is None else float(dt)
        if elapsed is not None and (not np.isfinite(elapsed) or elapsed <= 0.0):
            raise ValueError(f"dt must be finite and positive, got {dt}")
        properties = self._prepare_properties()
        if properties is None:
            return None

        pos, quat, vel, ang = self._link_states(properties.links, snapshot=snapshot)
        rotation = Rot.from_quat(quat[:, [1, 2, 3, 0]]).as_matrix()
        offsets_world = np.einsum('nij,nj->ni', rotation, properties.offsets)
        centers_world = pos + offsets_world
        velocities_world = vel + np.cross(ang, offsets_world)
        com_world = properties.masses @ centers_world / properties.measured_mass
        vel_world = properties.masses @ velocities_world / properties.measured_mass
        if elapsed is not None:
            if self._previous_com_world is not None:
                finite_difference = (com_world - self._previous_com_world) / elapsed
                if np.linalg.norm(vel_world) < 0.02 and np.linalg.norm(finite_difference) > 0.03:
                    vel_world = finite_difference
                    if not self._warned_stale_velocity:
                        print(
                            "[go2_mpc] arm payload link velocity readback is stale/zero; "
                            "using position-difference feedback"
                        )
                        self._warned_stale_velocity = True
            self._previous_com_world = com_world.copy()

        inertia_world = np.einsum('nij,njk,nlk->nil', rotation, properties.inertias, rotation).sum(axis=0)
        displacement = centers_world - com_world
        inertia_world += np.eye(3) * np.sum(properties.masses * np.sum(displacement**2, axis=1))
        inertia_world -= np.einsum('n,ni,nj->ij', properties.masses, displacement, displacement)

        mass_kg = self._mass_override if self._mass_override > 0.0 else properties.measured_mass
        return ArmPayloadSnapshot(
            mass_kg=float(mass_kg),
            com_world=np.asarray(com_world, dtype=float),
            vel_world=np.asarray(vel_world, dtype=float),
            inertia_world=inertia_world,
        )

    def apply(self, pin_model, *, dt: float | None = None, snapshot=None) -> None:
        payload = self.measure(dt=dt, snapshot=snapshot)
        if payload is None or payload.mass_kg <= 1e-9:
            return

        m_robot = float(pin_model.data.Ig.mass)
        com_robot = np.asarray(pin_model.pos_com_world, dtype=float).reshape(3)
        vel_robot = np.asarray(pin_model.vel_com_world, dtype=float).reshape(3)
        inertia_robot = np.asarray(pin_model.data.Ig.inertia, dtype=float).reshape(3, 3)

        m_payload = float(payload.mass_kg)
        com_payload = payload.com_world
        vel_payload = payload.vel_world
        inertia_payload = payload.inertia_world

        m_total = m_robot + m_payload
        com_total = (m_robot * com_robot + m_payload * com_payload) / m_total
        vel_total = (m_robot * vel_robot + m_payload * vel_payload) / m_total

        inertia_total = _shift_inertia(inertia_robot, m_robot, com_robot, com_total)
        inertia_total += _shift_inertia(inertia_payload, m_payload, com_payload, com_total)

        pin_model.data.Ig.mass = m_total
        pin_model.data.Ig.inertia = inertia_total
        pin_model.data.com[0] = com_total
        pin_model.data.vcom[0] = vel_total
        pin_model.pos_com_world = com_total
        pin_model.vel_com_world = vel_total

    def measure_com_body(self, go2_entity) -> np.ndarray | None:
        """Payload COM in GO2 base body frame (updates with arm joint angles)."""
        payload = self.measure()
        if payload is None:
            return None
        base = go2_entity.get_link("base")
        base_pos = _to_numpy_1d(base.get_pos())[:3]
        quat_xyzw = _quat_wxyz_to_xyzw(_to_numpy_1d(base.get_quat())[:4])
        rot_wb = Rot.from_quat(quat_xyzw).as_matrix()
        return rot_wb.T @ (payload.com_world - base_pos)


def backward_pitch_trim_rad(
    com_body: np.ndarray,
    *,
    gain_z: float,
    z_ref_m: float,
    max_trim_rad: float,
) -> float:
    """Nose-down pitch trim for backward walking with a high/folded arm payload."""
    com_z = float(com_body[2])
    excess_z = max(0.0, com_z - float(z_ref_m))
    trim = -float(gain_z) * excess_z
    return float(np.clip(trim, -abs(float(max_trim_rad)), 0.0))


def payload_pitch_trim_rad(com_body: np.ndarray, *, vx: float, config) -> float:
    """Body-frame payload COM based pitch reference trim."""
    com_x = float(com_body[0])
    com_z = float(com_body[2])
    vx_f = float(vx)
    if vx_f >= 0.05:
        kx = float(config.pitch_trim_gain_x_forward)
    elif vx_f <= -0.05:
        kx = float(config.pitch_trim_gain_x_backward)
    else:
        kx = 0.0
    kz = float(config.pitch_trim_gain_z)
    z_ref = float(config.pitch_trim_z_ref_m)
    pitch_max = abs(float(config.pitch_trim_max_rad))
    excess_z = max(0.0, com_z - z_ref)
    trim = kx * com_x + kz * excess_z
    return float(np.clip(trim, -pitch_max, pitch_max))
