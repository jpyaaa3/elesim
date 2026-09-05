"""Offline diagnostics for recorded Look-Aim-Grasp console logs."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Iterable

import numpy as np


_VECTOR = r"\[\s*([^\]]+)\]"
_PERCEPTION_RE = re.compile(rf"\[Perception\].*?camera={_VECTOR}\s*m\s+world={_VECTOR}\s*m")
_CONTROL_RE = re.compile(
    rf"\[Grasp-Ctrl\].*?dq_cmd={_VECTOR}\s+dq_meas={_VECTOR}.*?remain=([+-]?[0-9.]+)mm"
)
_BLIND_REMAIN_RE = re.compile(r"\[Grasp\]\s+blind extend\s+\|\s+axial_remain=([+-]?[0-9.]+)mm")
_LOOK_ERROR_RE = re.compile(r"look_err=([+-]?[0-9.]+)deg")
_LOOK_TOL_RE = re.compile(r"tol\s+([+-]?[0-9.]+)deg")


@dataclass(frozen=True)
class TraceIssue:
    code: str
    line_number: int
    message: str


@dataclass(frozen=True)
class PickReplayReport:
    perception_samples: int
    control_samples: int
    max_world_jump_m: float
    measured_motion_stalls: int
    remain_regressions: int
    blind_handoff_remain_m: float | None
    max_blind_look_error_deg: float | None
    outcome: str
    issues: tuple[TraceIssue, ...]

    @property
    def issue_codes(self) -> frozenset[str]:
        return frozenset(issue.code for issue in self.issues)


def _parse_vector(raw: str, *, expected: int) -> np.ndarray | None:
    try:
        values = np.asarray([float(part.strip()) for part in raw.split(",")], dtype=float)
    except ValueError:
        return None
    if values.shape != (expected,) or not np.all(np.isfinite(values)):
        return None
    return values


def _append_once(issues: list[TraceIssue], issue: TraceIssue) -> None:
    if issue.code not in {item.code for item in issues}:
        issues.append(issue)


def analyze_pick_log(
    text: str | Iterable[str],
    *,
    world_jump_warn_m: float = 0.03,
    measured_motion_ratio_min: float = 0.10,
    remain_regression_warn_m: float = 0.0015,
    blind_handoff_warn_m: float = 0.05,
    blind_look_error_warn_deg: float = 12.0,
) -> PickReplayReport:
    """Parse stable console fields and report causal warnings without rerunning Genesis."""
    lines = text.splitlines() if isinstance(text, str) else list(text)
    world_samples: list[np.ndarray] = []
    remain_samples: list[float] = []
    issues: list[TraceIssue] = []
    stalls = 0
    regressions = 0
    blind_remain: float | None = None
    blind_errors: list[float] = []
    outcome = "unknown"
    control_samples = 0

    for line_number, line in enumerate(lines, start=1):
        perception = _PERCEPTION_RE.search(line)
        if perception is not None:
            world = _parse_vector(perception.group(2), expected=3)
            if world is not None:
                if world_samples:
                    jump = float(np.linalg.norm(world - world_samples[-1]))
                    if jump > float(world_jump_warn_m):
                        _append_once(
                            issues,
                            TraceIssue(
                                "world_pose_jump",
                                line_number,
                                f"world-space object estimate jumped {jump * 1000.0:.1f} mm",
                            ),
                        )
                world_samples.append(world)

        control = _CONTROL_RE.search(line)
        if control is not None:
            command = _parse_vector(control.group(1), expected=4)
            measured = _parse_vector(control.group(2), expected=4)
            remain = float(control.group(3)) / 1000.0
            control_samples += 1
            if command is not None and measured is not None:
                command_norm = float(np.linalg.norm(command))
                measured_norm = float(np.linalg.norm(measured))
                if command_norm > 1e-6 and measured_norm < command_norm * float(measured_motion_ratio_min):
                    stalls += 1
                    _append_once(
                        issues,
                        TraceIssue(
                            "measured_motion_stall",
                            line_number,
                            "measured joint motion was negligible relative to the command",
                        ),
                    )
            if remain_samples and remain > remain_samples[-1] + float(remain_regression_warn_m):
                regressions += 1
                _append_once(
                    issues,
                    TraceIssue(
                        "approach_regression",
                        line_number,
                        f"remaining approach distance increased to {remain * 1000.0:.1f} mm",
                    ),
                )
            remain_samples.append(remain)

        blind_match = _BLIND_REMAIN_RE.search(line)
        if blind_match is not None:
            blind_remain = float(blind_match.group(1)) / 1000.0
            if blind_remain > float(blind_handoff_warn_m):
                _append_once(
                    issues,
                    TraceIssue(
                        "blind_handoff_long",
                        line_number,
                        f"perception stopped with {blind_remain * 1000.0:.1f} mm still remaining",
                    ),
                )

        look_match = _LOOK_ERROR_RE.search(line)
        if look_match is not None and "[Grasp]" in line:
            look_error = abs(float(look_match.group(1)))
            blind_errors.append(look_error)
            tolerance_match = _LOOK_TOL_RE.search(line)
            tolerance = (
                float(tolerance_match.group(1))
                if tolerance_match is not None
                else float(blind_look_error_warn_deg)
            )
            if look_error > tolerance:
                _append_once(
                    issues,
                    TraceIssue(
                        "blind_direction_error",
                        line_number,
                        f"blind approach direction error was {look_error:.1f} deg (limit {tolerance:.1f} deg)",
                    ),
                )

        lowered = line.lower()
        if "[grasp]" in lowered and "abort" in lowered:
            outcome = "aborted"
            _append_once(
                issues,
                TraceIssue("grasp_aborted", line_number, line.strip()),
            )
        elif "[grasp]" in lowered and ("done" in lowered or "claw closed" in lowered):
            if outcome != "aborted":
                outcome = "completed"

    max_jump = 0.0
    if len(world_samples) > 1:
        max_jump = max(
            float(np.linalg.norm(current - previous))
            for previous, current in zip(world_samples, world_samples[1:])
        )

    return PickReplayReport(
        perception_samples=len(world_samples),
        control_samples=control_samples,
        max_world_jump_m=max_jump,
        measured_motion_stalls=stalls,
        remain_regressions=regressions,
        blind_handoff_remain_m=blind_remain,
        max_blind_look_error_deg=max(blind_errors) if blind_errors else None,
        outcome=outcome,
        issues=tuple(issues),
    )


__all__ = ["PickReplayReport", "TraceIssue", "analyze_pick_log"]
