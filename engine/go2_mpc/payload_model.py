from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from engine.go2_mpc.genesis_pin_bridge import _quat_wxyz_to_xyzw, _to_numpy_1d


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


class ArmPayloadCompensator:
    """Estimate welded arm inertial properties from Genesis and patch PinGo2Model COM state."""

    def __init__(self, arm_entity, *, mass_override_kg: float = 0.0) -> None:
        self._arm = arm_entity
        self._mass_override = max(0.0, float(mass_override_kg))

    def measure(self) -> ArmPayloadSnapshot | None:
        link_samples: list[tuple[float, np.ndarray, np.ndarray, np.ndarray | None]] = []

        for link in self._arm.links:
            mass = link.inertial_mass
            if mass is None or float(mass) <= 1e-9:
                continue

            mass = float(mass)
            pos = _to_numpy_1d(link.get_pos())[:3]
            quat_xyzw = _quat_wxyz_to_xyzw(_to_numpy_1d(link.get_quat())[:4])
            rot = Rot.from_quat(quat_xyzw)
            rot_m = rot.as_matrix()
            inertial_pos = link.inertial_pos
            if inertial_pos is None:
                offset = np.zeros(3, dtype=float)
            else:
                offset = np.asarray(inertial_pos, dtype=float).reshape(3)
            offset_world = rot_m @ offset
            com_world = pos + offset_world

            vel = _to_numpy_1d(link.get_vel())[:3]
            ang = _to_numpy_1d(link.get_ang())[:3]
            vel_com = vel + np.cross(ang, offset_world)

            inertia_local = None
            if link.inertial_i is not None:
                inertia_local = rot_m @ np.asarray(link.inertial_i, dtype=float).reshape(3, 3) @ rot_m.T

            link_samples.append((mass, com_world, vel_com, inertia_local))

        if not link_samples:
            return None

        measured_mass = float(sum(m for m, _, _, _ in link_samples))
        com_world = sum(m * c for m, c, _, _ in link_samples) / measured_mass
        vel_world = sum(m * v for m, _, v, _ in link_samples) / measured_mass

        inertia_world = np.zeros((3, 3), dtype=float)
        for mass, com_link, _, inertia_link in link_samples:
            if inertia_link is not None:
                inertia_world += _shift_inertia(inertia_link, mass, com_link, com_world)
            else:
                s = com_link - com_world
                inertia_world += mass * (float(np.dot(s, s)) * np.eye(3) - np.outer(s, s))

        mass_kg = self._mass_override if self._mass_override > 0.0 else measured_mass
        return ArmPayloadSnapshot(
            mass_kg=float(mass_kg),
            com_world=np.asarray(com_world, dtype=float),
            vel_world=np.asarray(vel_world, dtype=float),
            inertia_world=inertia_world,
        )

    def apply(self, pin_model) -> None:
        payload = self.measure()
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
