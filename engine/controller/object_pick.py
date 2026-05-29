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
    EXTEND = "extend"
    DONE = "done"
    FAILED = "failed"


def grid_cell_center_uv(
    col: int,
    row: int,
    *,
    grid_cols: int = 2,
    grid_rows: int = 2,
    row0_is_top: bool = True,
) -> tuple[float, float]:
    """Normalized image UV center of a grid cell; row 0 = top."""
    c = int(col)
    r = int(row)
    if row0_is_top:
        row_idx = float(r)
    else:
        row_idx = float(grid_rows - 1 - r)
    u = 2.0 * (float(c) + 0.5) / float(max(grid_cols, 1)) - 1.0
    v = 2.0 * (row_idx + 0.5) / float(max(grid_rows, 1)) - 1.0
    return float(u), float(v)


def quadrant_fill_target_scale(fill_ratio: float, *, quadrants: int = 4) -> float:
    """Mask area / full image when object fills ``fill_ratio`` of one quadrant."""
    q = max(int(quadrants), 1)
    return float(max(0.0, min(1.0, float(fill_ratio) / float(q))))


@dataclass(frozen=True)
class PickConvergence:
    center_ok: bool
    scale_ok: bool
    u_err: float
    v_err: float
    scale: float


def pick_uv_deltas(
    obs: VisualObservation,
    *,
    cfg: PickConfig,
) -> tuple[float, float]:
    u = float(obs.center_uv[0])
    v = float(obs.center_uv[1])
    return u - float(cfg.target_uv_u), v - float(cfg.target_uv_v)


def pick_ready_for_extend(
    obs: VisualObservation,
    *,
    cfg: PickConfig,
    approach_steps: int = 0,
    scale_plateau: bool = False,
) -> tuple[bool, str]:
    """
    Extend when strictly aligned, or after approach effort when CSRT scale stalls
    (bbox area stops growing even as the arm advances).
    """
    conv = evaluate_pick_convergence(obs, cfg=cfg)
    if conv.center_ok and conv.scale_ok:
        return True, "aligned"
    du, dv = pick_uv_deltas(obs, cfg=cfg)
    loose = float(cfg.approach_loose_center_tol)
    center_loose = abs(du) <= loose and abs(dv) <= loose
    scale_floor = float(obs.scale) >= float(cfg.approach_min_scale)
    min_steps = max(1, int(cfg.approach_min_steps))
    if not scale_floor or not center_loose:
        return False, ""
    if bool(scale_plateau):
        return True, "scale_plateau"
    if int(approach_steps) >= min_steps:
        return True, "approach_steps"
    return False, ""


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
