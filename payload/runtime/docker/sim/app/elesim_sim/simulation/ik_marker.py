"""Display-only IK goal geometry, owned by each Genesis scene process."""
from __future__ import annotations

import numpy as np


class IkTargetMarker:
    def __init__(self):
        self.handles = []
        self.signature = None

    def update(self, scene, position, direction=None, *, visible=True):
        pos = None if position is None else np.asarray(position, dtype=float).reshape(3)
        vec = np.zeros(3) if direction is None else np.asarray(direction, dtype=float).reshape(3)
        valid = visible and pos is not None and np.isfinite(pos).all() and np.isfinite(vec).all()
        norm = float(np.linalg.norm(vec))
        vec = vec/norm if norm > 1e-9 else np.zeros(3)
        signature = tuple(np.r_[pos, vec]) if valid else None
        if signature == self.signature:
            return
        for handle in self.handles:
            scene.clear_debug_object(handle)
        self.handles.clear()
        self.signature = None
        if not valid:
            return
        color = (0.1, 1.0, 0.2, 0.95)
        self.handles.append(scene.draw_debug_sphere(pos=pos, radius=0.012, color=color))
        if norm > 1e-9:
            self.handles.append(scene.draw_debug_arrow(pos=pos, vec=vec*.09, radius=.003, color=color))
        self.signature = signature
