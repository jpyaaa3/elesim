from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np


class LegId(str, Enum):
    FL = "FL"
    FR = "FR"
    RL = "RL"
    RR = "RR"


ALL_LEGS: tuple[LegId, ...] = (LegId.FL, LegId.FR, LegId.RL, LegId.RR)


class LegPhase(str, Enum):
    STANCE = "stance"
    SWING = "swing"


@dataclass(frozen=True)
class Go2Command:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0

    def is_idle(self, threshold: float = 0.05) -> bool:
        return float(np.linalg.norm([self.vx, self.vy, self.yaw_rate])) < float(threshold)


@dataclass(frozen=True)
class FootTarget:
    leg: LegId
    pos_body: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "pos_body", np.asarray(self.pos_body, dtype=float).reshape(3))
