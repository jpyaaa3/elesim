from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class PitchLeadState:
    pitch_rate_filtered: float = 0.0
    pitch_rate_prev: float = 0.0
    pitch_acc_est: float = 0.0
    pitch_rate_lead: float = 0.0
    preview_dt_s: float = 0.0
    ok: bool = False
    reason: str = ""


class PitchLeadEstimator:
    """Pitch-rate lead using sim timestamp delta (worker period fallback)."""

    def __init__(self) -> None:
        self._prev_go2_base_timestamp_s: Optional[float] = None
        self._pitch_rate_prev: float = 0.0
        self._pitch_rate_filtered: float = 0.0
        self._initialized: bool = False

    def reset(self) -> None:
        self._prev_go2_base_timestamp_s = None
        self._pitch_rate_prev = 0.0
        self._pitch_rate_filtered = 0.0
        self._initialized = False

    @property
    def prev_go2_base_timestamp_s(self) -> Optional[float]:
        return self._prev_go2_base_timestamp_s

    def update(
        self,
        *,
        pitch_rate: float,
        go2_base_timestamp_s: float,
        worker_period_s: float,
        tau_s: float,
        lowpass_alpha: float,
    ) -> PitchLeadState:
        ts = float(go2_base_timestamp_s)
        if ts <= 0.0 or not math.isfinite(ts):
            return PitchLeadState(ok=False, reason="missing_ts")

        rate = float(pitch_rate)
        if not math.isfinite(rate):
            return PitchLeadState(ok=False, reason="missing_pitch_rate")

        dt = 0.0
        if self._prev_go2_base_timestamp_s is not None:
            dt = ts - float(self._prev_go2_base_timestamp_s)
        if dt <= 0.0 or not math.isfinite(dt):
            dt = float(max(worker_period_s, 1e-6))

        alpha = float(min(max(lowpass_alpha, 0.0), 1.0))
        if not self._initialized:
            rate_f = rate
            self._initialized = True
        else:
            rate_f = alpha * rate + (1.0 - alpha) * float(self._pitch_rate_filtered)

        if self._prev_go2_base_timestamp_s is None:
            acc = 0.0
            lead = rate_f
        else:
            acc = (rate_f - float(self._pitch_rate_prev)) / dt
            lead = rate_f + float(tau_s) * acc

        self._prev_go2_base_timestamp_s = ts
        self._pitch_rate_prev = rate_f
        self._pitch_rate_filtered = rate_f

        return PitchLeadState(
            pitch_rate_filtered=rate_f,
            pitch_rate_prev=float(self._pitch_rate_prev),
            pitch_acc_est=float(acc),
            pitch_rate_lead=float(lead),
            preview_dt_s=float(dt),
            ok=True,
            reason="",
        )


def resolve_pitch_rate(
    host,
    *,
    prev_pitch_rad: Optional[float],
    prev_go2_base_timestamp_s: Optional[float],
    worker_period_s: float,
) -> tuple[Optional[float], Optional[float]]:
    """Return (pitch_rate, pitch_rad) for lead estimator input."""
    if host is None:
        return None, None
    ts = float(getattr(host, "go2_base_timestamp_s", 0.0) or 0.0)
    ang = getattr(host, "go2_base_ang_vel_body", None)
    if ts > 0.0 and ang is not None and len(ang) >= 2:
        rate = float(ang[1])
        if math.isfinite(rate):
            return rate, None
    rpy = getattr(host, "go2_base_rpy", None)
    if rpy is None or len(rpy) < 2 or ts <= 0.0:
        return None, None
    pitch = float(rpy[1])
    if not math.isfinite(pitch):
        return None, None
    if prev_pitch_rad is None or prev_go2_base_timestamp_s is None:
        return 0.0, pitch
    dt = ts - float(prev_go2_base_timestamp_s)
    if dt <= 0.0 or not math.isfinite(dt):
        dt = float(max(worker_period_s, 1e-6))
    rate = (pitch - float(prev_pitch_rad)) / dt
    return float(rate), pitch


def extract_pitch_rate(host) -> Optional[float]:
    """Body pitch rate from host; fallback to RPY differentiation is done by caller."""
    if host is None:
        return None
    ang = getattr(host, "go2_base_ang_vel_body", None)
    if ang is not None and len(ang) >= 2:
        rate = float(ang[1])
        if math.isfinite(rate):
            return rate
    rpy = getattr(host, "go2_base_rpy", None)
    if rpy is not None and len(rpy) >= 2:
        pitch = float(rpy[1])
        if math.isfinite(pitch):
            return pitch
    return None
