"""Dynamics terms for the twelve-leg-DOF floating-base GO2 model.

An optional arm snapshot is treated as a rigid payload at its current pose.
This includes its mass, COM and inertia, but not internal arm acceleration.
Use a private model/data pair so dynamics queries cannot overwrite MPC state.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class ContactDynamicsTerms:
    mass_matrix: np.ndarray
    bias: np.ndarray
    foot_jacobians: np.ndarray
    foot_acceleration_bias: np.ndarray


class PinContactDynamics:
    def __init__(self, model, foot_frames, *, joint_armature: float):
        import pinocchio as pin

        if model.nq != 19 or model.nv != 18:
            raise ValueError('contact dynamics requires a floating base and twelve leg DOFs')
        if len(foot_frames) != 4 or any(f < 0 or f >= model.nframes for f in foot_frames):
            raise ValueError('contact dynamics requires four valid foot frames')
        if not np.isfinite(joint_armature) or joint_armature < 0.:
            raise ValueError('joint armature must be finite and nonnegative')
        self._pin = pin
        self.model = pin.Model(model)
        self.model.armature[6:] = float(joint_armature)
        self._bare_base = pin.Inertia(model.inertias[1])
        self._data = self.model.createData()
        self._frames = tuple(foot_frames)

    def terms(self, q, velocity, *, payload=None) -> ContactDynamicsTerms:
        pin, model, data = self._pin, self.model, self._data
        q, velocity = np.asarray(q, dtype=float), np.asarray(velocity, dtype=float)
        if (q.shape != (19,) or velocity.shape != (18,)
                or not np.all(np.isfinite(q)) or not np.all(np.isfinite(velocity))
                or abs(np.linalg.norm(q[3:7])-1.) > 1e-6):
            raise ValueError('contact dynamics requires finite q/velocity and a unit quaternion')
        # Start from the synchronized bare-body property on every query.
        # Repeated snapshots must not accumulate the arm into the base.
        base_inertia = self._bare_base
        if payload is not None:
            mass = float(payload.mass_kg)
            com = np.asarray(payload.com_world, dtype=float)
            inertia = np.asarray(payload.inertia_world, dtype=float)
            if (not np.isfinite(mass) or mass <= 0. or com.shape != (3,)
                    or inertia.shape != (3, 3) or not np.all(np.isfinite(com))
                    or not np.all(np.isfinite(inertia))):
                raise ValueError('contact dynamics received invalid rigid payload')
            rotation = Rotation.from_quat(q[3:7]).as_matrix()
            base_inertia = base_inertia + pin.Inertia(
                mass, rotation.T @ (com-q[:3]), rotation.T @ inertia @ rotation)
        model.inertias[1] = base_inertia

        upper = np.triu(np.asarray(pin.crba(model, data, q), dtype=float))
        mass_matrix = upper + np.triu(upper, 1).T
        bias = np.asarray(pin.nonLinearEffects(model, data, q, velocity), dtype=float).copy()
        pin.forwardKinematics(model, data, q, velocity, np.zeros(model.nv))
        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        jacobians = np.array([pin.getFrameJacobian(model, data, frame,
                              pin.LOCAL_WORLD_ALIGNED)[:3] for frame in self._frames])
        gamma = np.array([pin.getFrameClassicalAcceleration(model, data, frame,
                          pin.LOCAL_WORLD_ALIGNED).linear for frame in self._frames])
        return ContactDynamicsTerms(mass_matrix, bias, jacobians, gamma)
