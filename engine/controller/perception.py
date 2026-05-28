from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PerceptionState:
    """Typed placeholder for future host/ctrl perception state."""

    object_world_xyz: Optional[tuple[float, float, float]] = None
    object_label: str = ""
    timestamp_s: float = 0.0
