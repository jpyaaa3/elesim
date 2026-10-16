"""Convert prescribed ground forces to actuator torques for grounded feet.

This is a small floating-base inverse dynamics solve, not a force optimizer.
The caller supplies an 18-DOF model, world-aligned linear foot Jacobians and
their classical zero-acceleration bias. Only confirmed support may carry force.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ContactInverseDynamicsResult:
    acceleration: np.ndarray
    torque: np.ndarray


def contact_inverse_dynamics(
    mass_matrix, bias, foot_jacobians, foot_acceleration_bias, contacts, forces,
    *, swing_torque, passive_torque, contact_acceleration=None,
) -> ContactInverseDynamicsResult:
    """Honor base dynamics, prescribed swing torques and stance acceleration.

    ``M a + h = S.T (tau + passive) + J.T f`` uses positive ground reaction
    forces on the robot. ``passive_torque`` is the signed physical motor load
    (e.g. ``-damping * velocity``), not a positive damping coefficient. For each
    support leg the three joint equations determine actuator torque; the other
    equations and ``J a + Jdot v = contact_acceleration`` determine acceleration.

    Limits, contact confirmation and force projection belong to the caller.
    A singular solve is an error; it must not silently create airborne support.
    """
    def array(value, shape, name):
        result = np.asarray(value, dtype=float)
        if result.shape != shape or not np.all(np.isfinite(result)):
            raise ValueError(f'{name} must contain finite values with shape {shape}')
        return result

    mass = array(mass_matrix, (18, 18), 'mass matrix')
    h = array(bias, (18,), 'dynamics bias')
    jacobians = array(foot_jacobians, (4, 3, 18), 'foot Jacobians')
    gamma = array(foot_acceleration_bias, (4, 3), 'foot acceleration bias')
    support = array(contacts, (4,), 'contacts')
    if not np.all((support == 0.) | (support == 1.)):
        raise ValueError('contacts must be confirmed binary support')
    support = support.astype(bool)
    force = array(forces, (4, 3), 'ground forces')
    swing = array(swing_torque, (12,), 'swing torque')
    passive = array(passive_torque, (12,), 'passive torque')
    target = (np.zeros((4, 3)) if contact_acceleration is None else
              array(contact_acceleration, (4, 3), 'contact acceleration'))
    if np.any(force[~support] != 0.) or np.any(force[:, 2] < 0.):
        raise ValueError('ground force requires confirmed support and nonnegative normal')
    if not np.allclose(mass, mass.T, rtol=1e-10, atol=1e-12):
        raise ValueError('mass matrix must be symmetric')

    stance_dofs = np.repeat(support, 3)
    free_rows = np.r_[np.ones(6, dtype=bool), ~stance_dofs]
    contact_j = jacobians[support].reshape(-1, 18)
    external = contact_j.T @ force[support].ravel()
    known_actuation = np.r_[np.zeros(6), swing + passive]
    system = np.vstack((mass[free_rows], contact_j))
    rhs = np.r_[(external + known_actuation - h)[free_rows],
                (target - gamma)[support].ravel()]
    try:
        acceleration = np.linalg.solve(system, rhs)
    except np.linalg.LinAlgError as exc:
        raise RuntimeError('contact inverse dynamics is singular') from exc
    actuator = (mass @ acceleration + h - external)[6:] - passive
    actuator[~stance_dofs] = swing[~stance_dofs]
    if not np.all(np.isfinite(acceleration)) or not np.all(np.isfinite(actuator)):
        raise RuntimeError('contact inverse dynamics produced nonfinite torque')
    return ContactInverseDynamicsResult(acceleration=acceleration, torque=actuator)
