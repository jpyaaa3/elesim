"""Object pick phase helpers and convergence checks."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from engine.config_loader import PickConfig

from .perception import VisualObservation


class ObjectPickPhase(str, Enum):
    IDLE = "idle"
    ACQUIRE = "acquire"
    CENTER = "center"
    APPROACH = "approach"
    DONE = "done"
    FAILED = "failed"


@dataclass(frozen=True)
class PickConvergence:
    center_ok: bool
    scale_ok: bool
    u_err: float
    v_err: float
    scale: float


def evaluate_pick_convergence(
    obs: VisualObservation,
    *,
    cfg: PickConfig,
) -> PickConvergence:
    u = float(obs.center_uv[0])
    v = float(obs.center_uv[1])
    scale = float(obs.scale)
    center_tol = float(cfg.center_tol)
    scale_tol = float(cfg.scale_tol)
    target_scale = float(cfg.target_scale)
    tu = float(cfg.target_uv_u)
    tv = float(cfg.target_uv_v)
    u_delta = u - tu
    v_delta = v - tv
    return PickConvergence(
        center_ok=abs(u_delta) <= center_tol and abs(v_delta) <= center_tol,
        scale_ok=scale >= target_scale - scale_tol,
        u_err=u,
        v_err=v,
        scale=scale,
    )
