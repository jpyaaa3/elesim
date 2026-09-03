"""Sequential phase runner for pilot-owned Pick workflows."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable


@dataclass(frozen=True)
class PickWorkflowPhase:
    label: str
    state_phase: str
    start: Callable[[], None]


@dataclass(frozen=True)
class PickWorkflowResult:
    success: bool
    reason: str
    phase: str = ""
    detail: str = ""

    @classmethod
    def completed(cls) -> "PickWorkflowResult":
        return cls(success=True, reason="completed")


def run_pick_workflow(
    phases: Iterable[PickWorkflowPhase],
    *,
    timeout_s: float,
    begin_phase: Callable[[PickWorkflowPhase], None],
    wait_phase: Callable[[str, float], bool],
    failed: Callable[[], bool],
    cancelled: Callable[[], bool],
) -> PickWorkflowResult:
    """Run named phases in order and classify every early exit."""
    for phase in phases:
        if cancelled():
            return PickWorkflowResult(False, "cancelled", phase.label)

        begin_phase(phase)
        try:
            phase.start()
        except Exception as exc:
            return PickWorkflowResult(False, "exception", phase.label, str(exc))

        if failed():
            return PickWorkflowResult(False, "failed", phase.label)
        if cancelled():
            return PickWorkflowResult(False, "cancelled", phase.label)
        if wait_phase(phase.label, float(timeout_s)):
            continue
        if failed():
            return PickWorkflowResult(False, "failed", phase.label)
        if cancelled():
            return PickWorkflowResult(False, "cancelled", phase.label)
        return PickWorkflowResult(False, "timeout", phase.label)

    return PickWorkflowResult.completed()


__all__ = ["PickWorkflowPhase", "PickWorkflowResult", "run_pick_workflow"]
