"""Pick/Look/Ready phase timing for bottleneck profiling."""

from __future__ import annotations

import os
from contextlib import contextmanager
from dataclasses import dataclass, field
from time import perf_counter
from typing import Any, Generator, Optional

_FK_ORIGINAL: Any = None
_FK_COUNT: int = 0


def enabled() -> bool:
    return os.environ.get("ELESIM_PROFILE_PICK", "").strip() == "1"


def install_fk_counter() -> None:
    """Count FK grasp-position calls while profiling (no-op if already installed)."""
    global _FK_ORIGINAL, _FK_COUNT
    if _FK_ORIGINAL is not None:
        return
    from engine.robot.arm.iklib import kinematics as ik_kin

    _FK_ORIGINAL = ik_kin._forward_grasp_world

    def _counting_forward_grasp_world(context: dict, q4: Any) -> Any:
        global _FK_COUNT
        _FK_COUNT += 1
        return _FK_ORIGINAL(context, q4)

    ik_kin._forward_grasp_world = _counting_forward_grasp_world  # type: ignore[assignment]


def uninstall_fk_counter() -> None:
    global _FK_ORIGINAL, _FK_COUNT
    if _FK_ORIGINAL is None:
        return
    from engine.robot.arm.iklib import kinematics as ik_kin

    ik_kin._forward_grasp_world = _FK_ORIGINAL
    _FK_ORIGINAL = None
    _FK_COUNT = 0


def reset_fk_count() -> None:
    global _FK_COUNT
    _FK_COUNT = 0


def fk_call_count() -> int:
    return int(_FK_COUNT)


@dataclass
class PickPhaseProfile:
    phase: str = ""
    t_total_s: float = 0.0
    t_candidate_build_s: float = 0.0
    t_resolve_s: float = 0.0
    t_solve_position_s: float = 0.0
    t_align_s: float = 0.0
    t_view_eval_s: float = 0.0
    t_host_apply_s: float = 0.0
    t_settle_s: float = 0.0
    candidates_evaluated: int = 0
    resolve_reason: str = ""
    ik_calls: int = 0
    fk_calls: int = 0
    success: bool = False
    extra: dict[str, Any] = field(default_factory=dict)


class PickTimingCollector:
    """Accumulates wall-time spans keyed by name."""

    def __init__(self) -> None:
        self._spans: dict[str, float] = {}
        self.candidates_evaluated: int = 0
        self.ik_calls: int = 0
        self.resolve_reason: str = ""
        self.extra: dict[str, Any] = {}

    def add(self, name: str, dt_s: float) -> None:
        self._spans[name] = float(self._spans.get(name, 0.0)) + float(dt_s)

    def get(self, name: str, default: float = 0.0) -> float:
        return float(self._spans.get(name, default))

    @contextmanager
    def span(self, name: str) -> Generator[None, None, None]:
        t0 = perf_counter()
        try:
            yield
        finally:
            self.add(name, perf_counter() - t0)

    def to_profile(
        self,
        *,
        phase: str,
        t_total_s: float,
        t_host_apply_s: float = 0.0,
        t_settle_s: float = 0.0,
        success: bool = False,
    ) -> PickPhaseProfile:
        t_resolve = self.get("resolve_grid")
        if t_resolve <= 0.0:
            t_resolve = self.get("resolve_single")
        return PickPhaseProfile(
            phase=str(phase),
            t_total_s=float(t_total_s),
            t_candidate_build_s=self.get("candidate_build"),
            t_resolve_s=float(t_resolve),
            t_solve_position_s=self.get("solve_position"),
            t_align_s=self.get("align_direction"),
            t_view_eval_s=self.get("view_eval"),
            t_host_apply_s=float(t_host_apply_s),
            t_settle_s=float(t_settle_s),
            candidates_evaluated=int(self.candidates_evaluated),
            resolve_reason=str(self.resolve_reason),
            ik_calls=int(self.ik_calls),
            fk_calls=fk_call_count(),
            success=bool(success),
            extra=dict(self.extra),
        )


def format_report(profile: PickPhaseProfile) -> str:
    lines = [
        f"[Profile] {profile.phase} | success={profile.success} reason={profile.resolve_reason or '-'}",
        f"  total          {profile.t_total_s * 1000.0:8.1f} ms",
        f"  candidate_build {profile.t_candidate_build_s * 1000.0:7.1f} ms",
        f"  resolve        {profile.t_resolve_s * 1000.0:8.1f} ms  (evaluated={profile.candidates_evaluated} ik_calls={profile.ik_calls})",
        f"    solve_position {profile.t_solve_position_s * 1000.0:7.1f} ms",
        f"    align_direction {profile.t_align_s * 1000.0:7.1f} ms",
        f"    view_eval      {profile.t_view_eval_s * 1000.0:7.1f} ms",
        f"  host_apply     {profile.t_host_apply_s * 1000.0:8.1f} ms",
        f"  settle_poll    {profile.t_settle_s * 1000.0:8.1f} ms",
        f"  fk_calls       {profile.fk_calls}",
    ]
    return "\n".join(lines)


__all__ = [
    "PickPhaseProfile",
    "PickTimingCollector",
    "enabled",
    "fk_call_count",
    "format_report",
    "install_fk_counter",
    "reset_fk_count",
    "uninstall_fk_counter",
]
