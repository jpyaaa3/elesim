"""Local first-control-step dynamics through inverse torque and actuation.

The matrices are rebuilt from the current floating-base/contact model. They
predict actuator clipping, torque history and force filtering explicitly.
Future contact changes and configuration changes are not predicted here.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .contact_inverse_dynamics import contact_inverse_dynamics

_PARAMETERS = (
    ('torque_from_force', (12, 12)), ('torque_bias', (12,)),
    ('angular_from_torque', (3, 12)), ('angular_bias', (3,)),
    ('force_from_torque', (12, 12)), ('force_bias', (12,)),
    ('torque_limits', (12,)), ('previous_torque', (12,)),
    ('torque_alpha', (12,)), ('previous_forces', (12,)), ('force_alpha', ()),
    ('physical_friction', ()),
)


def apply_actuator_prediction(array, forces, parameters):
    """Shared NumPy/JAX arithmetic for the fixed-shape local model."""
    t_map, t_bias, a_map, a_bias, f_map, f_bias, limits, history, alpha, old_f, f_alpha, _ = parameters
    force = f_alpha*forces.reshape(12)+(1.-f_alpha)*old_f
    requested = t_map @ force+t_bias
    applied = alpha*array.clip(requested, -limits, limits)+(1.-alpha)*history
    ground = (f_map @ applied+f_bias).reshape(4, 3)
    return requested, applied, ground, a_map @ applied+a_bias


def empty_actuator_parameters():
    """Keep one JIT signature when the optional local model is disabled."""
    return tuple(np.zeros(shape, dtype=np.float32) for _, shape in _PARAMETERS)


@dataclass(frozen=True)
class ActuatorPrediction:
    requested_torque: np.ndarray
    applied_torque: np.ndarray
    forces: np.ndarray
    angular_acceleration: np.ndarray


@dataclass(frozen=True)
class FirstStageActuation:
    torque_from_force: np.ndarray
    torque_bias: np.ndarray
    angular_from_torque: np.ndarray
    angular_bias: np.ndarray
    force_from_torque: np.ndarray
    force_bias: np.ndarray
    torque_limits: np.ndarray
    previous_torque: np.ndarray
    torque_alpha: np.ndarray
    previous_forces: np.ndarray
    force_alpha: float
    contacts: np.ndarray
    physical_friction: float = .8

    def predict(self, forces) -> ActuatorPrediction:
        force = np.asarray(forces, dtype=float)
        if (force.shape != (4, 3) or not np.all(np.isfinite(force))
                or np.any(force[~self.contacts] != 0.) or np.any(force[:, 2] < 0.)):
            raise ValueError('first-stage force requires finite confirmed support')
        return ActuatorPrediction(*apply_actuator_prediction(np, force, self.as_arrays(dtype=float)))

    def as_arrays(self, *, dtype=np.float32):
        """Fixed-shape JAX payload; order is shared with the local predictor."""
        values = []
        for name, shape in _PARAMETERS:
            value = np.asarray(getattr(self, name), dtype=dtype)
            if value.shape != shape or not np.all(np.isfinite(value)):
                raise ValueError(f'first-stage {name} must be finite with shape {shape}')
            values.append(value.copy())
        if (np.any(self.torque_limits <= 0.) or np.any((self.torque_alpha < 0.) | (self.torque_alpha > 1.))
                or not 0. <= self.force_alpha <= 1. or self.physical_friction <= 0.):
            raise ValueError('first-stage limits and filter coefficients are invalid')
        return tuple(values)


def build_actuator_prediction(terms, contacts, *, swing_torque, passive_torque,
                             torque_limits, previous_torque, torque_alpha,
                             previous_forces, force_alpha, physical_friction=.8) -> FirstStageActuation:
    def finite(value, shape, name):
        result = np.asarray(value, dtype=float)
        if result.shape != shape or not np.all(np.isfinite(result)):
            raise ValueError(f'{name} must be finite with shape {shape}')
        return result.copy()

    if not np.isfinite(force_alpha) or not 0. <= force_alpha <= 1.:
        raise ValueError('force_alpha must be finite within [0, 1]')
    if not np.isfinite(physical_friction) or physical_friction <= 0.:
        raise ValueError('physical_friction must be finite and positive')
    limits = finite(torque_limits, (12,), 'torque limits')
    history = finite(previous_torque, (12,), 'previous torque')
    alpha = finite(torque_alpha, (12,), 'torque_alpha')
    past_forces = finite(previous_forces, (4, 3), 'previous forces')
    if np.any(limits <= 0.) or np.any((alpha < 0.) | (alpha > 1.)):
        raise ValueError('torque limits must be positive and torque_alpha within [0, 1]')
    # This also validates mass, bias, kinematics, contacts and known torques.
    base = contact_inverse_dynamics(terms.mass_matrix, terms.bias, terms.foot_jacobians,
        terms.foot_acceleration_bias, contacts, np.zeros((4, 3)),
        swing_torque=swing_torque, passive_torque=passive_torque)
    support = np.asarray(contacts, dtype=bool)
    if np.any(past_forces[~support] != 0.) or np.any(past_forces[:, 2] < 0.):
        raise ValueError('previous forces must match confirmed support')
    stance = np.repeat(support, 3)
    mass = np.asarray(terms.mass_matrix, dtype=float)
    j = np.asarray(terms.foot_jacobians)[support].reshape(-1, 18)
    all_j = np.asarray(terms.foot_jacobians).reshape(12, 18)
    generalized = all_j.T*stance[None, :]
    free_rows = np.r_[np.ones(6, dtype=bool), ~stance]
    inverse_system = np.vstack((mass[free_rows], j))
    acceleration_from_force = np.linalg.solve(inverse_system,
        np.vstack((generalized[free_rows], np.zeros((len(j), 12)))))
    torque_from_force = mass[6:] @ acceleration_from_force-generalized[6:]
    torque_from_force[~stance] = 0.

    # Independent rigid-contact forward dynamics maps applied actuator torque
    # to base angular acceleration and the actual reaction forces.
    forward = np.block([[mass, -j.T], [j, np.zeros((len(j), len(j)))]])
    torque_rhs = np.vstack((np.zeros((6, 12)), np.eye(12), np.zeros((len(j), 12))))
    forward_map = np.linalg.solve(forward, torque_rhs)
    rhs = np.r_[np.r_[np.zeros(6), np.asarray(passive_torque)]-terms.bias,
                -np.asarray(terms.foot_acceleration_bias)[support].ravel()]
    forward_bias = np.linalg.solve(forward, rhs)
    force_map, force_bias = np.zeros((12, 12)), np.zeros(12)
    force_map[stance] = forward_map[18:]
    force_bias[stance] = forward_bias[18:]
    return FirstStageActuation(torque_from_force, base.torque.copy(),
        forward_map[3:6].copy(), forward_bias[3:6].copy(), force_map, force_bias,
        limits, history, alpha, past_forces.ravel(), float(force_alpha), support.copy(), float(physical_friction))
