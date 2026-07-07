from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

PHASE_SOURCE_GO2 = "go2_gait_phase"
PHASE_SOURCE_SIM = "sim_time_mod_period"
PHASE_SOURCE_WALL = "wall_time_from_run_start"


def resolve_gait_period_s(*, gait_period_s: float, gait_hz: float) -> float:
    if float(gait_period_s) > 0.0:
        return float(gait_period_s)
    if float(gait_hz) > 0.0:
        return 1.0 / float(gait_hz)
    return 0.0


def resolve_gait_phase(
    *,
    host_gait_phase: float | None,
    sim_time_s: float,
    wall_time_s: float,
    wall_t0_s: float,
    gait_period_s: float,
    phase_offset: float,
) -> tuple[float | None, str]:
    if host_gait_phase is not None and math.isfinite(host_gait_phase):
        return float(host_gait_phase) % 1.0, PHASE_SOURCE_GO2
    period = float(gait_period_s)
    sim_t = float(sim_time_s)
    if sim_t > 0.0 and period > 0.0:
        return (sim_t % period) / period, PHASE_SOURCE_SIM
    wall_t = float(wall_time_s)
    t0 = float(wall_t0_s)
    if wall_t > t0 and period > 0.0:
        return ((wall_t - t0) / period + float(phase_offset)) % 1.0, PHASE_SOURCE_WALL
    return None, ""


def phase_future(phase_now: float, *, horizon_s: float, period_s: float) -> float:
    period = float(period_s)
    if period <= 0.0:
        return float(phase_now) % 1.0
    return (float(phase_now) + float(horizon_s) / period) % 1.0


def _wrap_phase(phase: float) -> float:
    return float(phase) % 1.0


def _interp_circular(values: np.ndarray, phase: float) -> float:
    n = int(values.size)
    if n <= 0:
        return 0.0
    p = _wrap_phase(phase)
    pos = p * n
    i0 = int(math.floor(pos)) % n
    i1 = (i0 + 1) % n
    frac = pos - math.floor(pos)
    return float((1.0 - frac) * values[i0] + frac * values[i1])


def fill_empty_bins(values: np.ndarray, counts: np.ndarray) -> np.ndarray:
    out = np.asarray(values, dtype=float).copy()
    n = out.size
    if n == 0:
        return out
    if np.all(counts > 0):
        return out
    if not np.any(counts > 0):
        raise ValueError("all template bins empty")
    filled = counts > 0
    for _ in range(n):
        changed = False
        for i in range(n):
            if filled[i]:
                continue
            left = (i - 1) % n
            right = (i + 1) % n
            if filled[left] and filled[right]:
                out[i] = 0.5 * (out[left] + out[right])
                filled[i] = True
                changed = True
            elif filled[left]:
                out[i] = out[left]
                filled[i] = True
                changed = True
            elif filled[right]:
                out[i] = out[right]
                filled[i] = True
                changed = True
        if not changed:
            break
    if not np.all(filled):
        mean_v = float(np.mean(out[filled]))
        out[~filled] = mean_v
    return out


@dataclass(frozen=True)
class GaitPhaseTemplate:
    metadata: dict[str, Any]
    u_template: np.ndarray
    v_template: np.ndarray
    sample_count: np.ndarray
    u_std: np.ndarray
    v_std: np.ndarray

    @property
    def num_bins(self) -> int:
        return int(self.u_template.size)

    @property
    def gait_period_s(self) -> float:
        return float(self.metadata.get("gait_period_s", 0.0))

    @property
    def phase_source(self) -> str:
        return str(self.metadata.get("phase_source", ""))


@dataclass(frozen=True)
class GaitPreviewDelta:
    phase_now: float
    phase_future: float
    d_now: np.ndarray
    d_future: np.ndarray
    preview_term: np.ndarray
    ok: bool
    reason: str


class GaitPhasePreviewModel:
    def __init__(self, template: GaitPhaseTemplate) -> None:
        self.template = template
        counts = np.asarray(template.sample_count, dtype=float)
        u = fill_empty_bins(np.asarray(template.u_template, dtype=float), counts)
        v = fill_empty_bins(np.asarray(template.v_template, dtype=float), counts)
        self._u = u
        self._v = v

    @classmethod
    def load(cls, path: str | Path) -> GaitPhasePreviewModel:
        return cls(load_template(path))

    def lookup(self, phase: float) -> tuple[float, float]:
        p = _wrap_phase(phase)
        return _interp_circular(self._u, p), _interp_circular(self._v, p)

    def preview_delta(
        self,
        phase_now: float,
        *,
        scale: float,
        horizon_s: float,
        period_s: float,
    ) -> GaitPreviewDelta:
        try:
            p_now = _wrap_phase(phase_now)
            p_fut = phase_future(p_now, horizon_s=horizon_s, period_s=period_s)
            u_now, v_now = self.lookup(p_now)
            u_fut, v_fut = self.lookup(p_fut)
            d_now = np.array([u_now, v_now], dtype=float)
            d_future = np.array([u_fut, v_fut], dtype=float)
            term = float(scale) * (d_future - d_now)
            return GaitPreviewDelta(
                phase_now=float(p_now),
                phase_future=float(p_fut),
                d_now=d_now,
                d_future=d_future,
                preview_term=term,
                ok=True,
                reason="",
            )
        except (ValueError, FloatingPointError) as exc:
            return GaitPreviewDelta(
                phase_now=float(phase_now) % 1.0,
                phase_future=float(phase_now) % 1.0,
                d_now=np.zeros(2, dtype=float),
                d_future=np.zeros(2, dtype=float),
                preview_term=np.zeros(2, dtype=float),
                ok=False,
                reason=f"template_lookup_fail:{exc.__class__.__name__}",
            )


def load_template(path: str | Path) -> GaitPhaseTemplate:
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"gait template not found: {p}")
    payload = json.loads(p.read_text(encoding="utf-8"))
    meta = dict(payload.get("metadata") or {})
    u = np.asarray(payload.get("u_template") or [], dtype=float)
    v = np.asarray(payload.get("v_template") or [], dtype=float)
    counts = np.asarray(payload.get("sample_count") or [], dtype=float)
    u_std = np.asarray(payload.get("u_std") or np.zeros_like(u), dtype=float)
    v_std = np.asarray(payload.get("v_std") or np.zeros_like(v), dtype=float)
    if u.size == 0 or v.size == 0 or u.size != v.size:
        raise ValueError("invalid template: u_template/v_template size mismatch or empty")
    if counts.size != u.size:
        counts = np.zeros(u.size, dtype=float)
    return GaitPhaseTemplate(
        metadata=meta,
        u_template=u,
        v_template=v,
        sample_count=counts,
        u_std=u_std,
        v_std=v_std,
    )
