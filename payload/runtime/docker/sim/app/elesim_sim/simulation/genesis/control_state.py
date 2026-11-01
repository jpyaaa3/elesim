"""Owned, current-tick state shared by the GO2 bridge and payload estimator."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class EntityControlState:
    entity: object
    pos: np.ndarray
    quat: np.ndarray
    vel: np.ndarray
    ang: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    contact_force: np.ndarray


class EntityControlStateReader:
    """Read public authored-frame getters, then synchronize with the host once.

    All entity links/DOFs are contiguous in Genesis. Reading that small range
    avoids rebuilding GPU index arrays for each field. Select body/payload/leg
    rows on the CPU after the single transfer. Pose and velocity stay current.
    Contacts come from the latest completed physics step: Genesis clears them
    even when a position setter only moves the arm. Explicit reset discards
    that sample. Never use the solver's uncorrected internal link frames.
    """

    _GETTERS = ('get_links_pos', 'get_links_quat', 'get_links_vel', 'get_links_ang',
                'get_dofs_position', 'get_dofs_velocity', 'get_links_net_contact_force')

    def __init__(self, entity) -> None:
        self.entity = entity
        self._physics_contact_force = None
        links, dofs = int(entity.n_links), int(entity.n_dofs)
        if links <= 0 or dofs <= 0:
            raise ValueError('control state requires a built entity with links and DOFs')
        shapes = ((links, 3), (links, 4), (links, 3), (links, 3), (dofs,), (dofs,), (links, 3))
        offset, fields = 0, []
        for name, getter, shape in zip(tuple(EntityControlState.__dataclass_fields__)[1:], self._GETTERS, shapes):
            end = offset + int(np.prod(shape))
            fields.append((name, getter, shape, slice(offset, end)))
            offset = end
        self._fields = tuple(fields)

    def capture_physics_contacts(self) -> None:
        """Own the completed contact solve before any kinematic pose writes."""
        value = self.entity.get_links_net_contact_force()
        if tuple(value.shape) != (int(self.entity.n_links), 3):
            raise ValueError('physics contact forces must have shape (n_links, 3)')
        self._physics_contact_force = (value.detach().clone() if hasattr(value, 'detach')
                                       else np.array(value, copy=True))

    def clear_physics_contacts(self) -> None:
        self._physics_contact_force = None

    def contact_forces(self):
        if self._physics_contact_force is not None:
            return self._physics_contact_force
        return self.entity.get_links_net_contact_force()

    @classmethod
    def for_entity(cls, entity):
        # Minimal entities used by other integrations retain their individual
        # getter path. A supported entity's malformed readback must still fail.
        if not all(callable(getattr(entity, name, None)) for name in cls._GETTERS):
            return None
        if not hasattr(entity, 'n_links') or not hasattr(entity, 'n_dofs'):
            return None
        return cls(entity)

    def read(self) -> EntityControlState:
        values = []
        for name, getter, shape, _ in self._fields:
            value = self.contact_forces() if name == 'contact_force' else getattr(self.entity, getter)()
            if tuple(value.shape) != shape:
                raise ValueError(f'control state {name} must have shape {shape}, got {tuple(value.shape)}')
            values.append(value)
        if hasattr(values[0], 'detach'):
            import torch

            # cat owns storage even when a public getter is a zero-copy view.
            # The blocking host transfer completes all seven state reads.
            packet = torch.cat([value.detach().reshape(-1) for value in values]).cpu().numpy()
        else:
            packet = np.concatenate([np.asarray(value).reshape(-1) for value in values])
        packet = np.asarray(packet, dtype=float)
        packet.setflags(write=False)
        return EntityControlState(self.entity, *(packet[span].reshape(shape)
            for _, _, shape, span in self._fields))
