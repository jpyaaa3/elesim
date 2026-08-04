"""Main-thread simulation controls fed by the protocol endpoint."""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Callable

from elesim_protocol import (
    SimulationCommandRequest,
    SimulationResultPayload,
    SimulationStatusPayload,
)


_CAMERA_COMMANDS = frozenset({"orbit", "pan", "zoom", "reset_view"})


@dataclass(frozen=True)
class SimulationOperatorCommand:
    request_ids: tuple[str, ...]
    session_id: str
    ui_id: str
    command: str
    arguments: dict[str, Any]

    @classmethod
    def from_request(
        cls,
        request: SimulationCommandRequest,
        *,
        ui_id: str,
    ) -> "SimulationOperatorCommand":
        return cls(
            request_ids=(request.request_id,),
            session_id=request.session_id,
            ui_id=str(ui_id),
            command=request.command,
            arguments=dict(request.arguments),
        )


@dataclass(frozen=True)
class PendingSimulationResult:
    target_id: str
    payload: SimulationResultPayload

    @property
    def request_id(self) -> str:
        return self.payload.request_id


class SimulationOperatorMailbox:
    """Bounded non-blocking command queue shared by DDS and Genesis threads."""

    def __init__(self, *, max_pending: int = 128) -> None:
        self.max_pending = max(1, int(max_pending))
        self._lock = threading.Lock()
        self._pending: deque[SimulationOperatorCommand] = deque()
        self._results: deque[PendingSimulationResult] = deque()

    def enqueue(self, command: SimulationOperatorCommand) -> bool:
        with self._lock:
            if self._pending and self._can_coalesce(self._pending[-1], command):
                self._pending[-1] = self._coalesce(self._pending[-1], command)
                return True
            if len(self._pending) >= self.max_pending:
                return False
            self._pending.append(command)
            return True

    @staticmethod
    def _can_coalesce(
        previous: SimulationOperatorCommand,
        current: SimulationOperatorCommand,
    ) -> bool:
        return (
            current.command in {"orbit", "pan", "zoom"}
            and previous.command == current.command
            and previous.session_id == current.session_id
            and previous.ui_id == current.ui_id
        )

    @staticmethod
    def _coalesce(
        previous: SimulationOperatorCommand,
        current: SimulationOperatorCommand,
    ) -> SimulationOperatorCommand:
        if current.command in {"orbit", "pan"}:
            arguments = {
                key: max(-2.0, min(2.0, float(previous.arguments[key]) + float(current.arguments[key])))
                for key in ("dx", "dy")
            }
        else:
            arguments = {
                "delta": max(
                    -2.0,
                    min(2.0, float(previous.arguments["delta"]) + float(current.arguments["delta"])),
                )
            }
        return replace(
            previous,
            request_ids=previous.request_ids + current.request_ids,
            arguments=arguments,
        )

    def drain(self, *, max_items: int = 64) -> list[SimulationOperatorCommand]:
        with self._lock:
            count = min(max(1, int(max_items)), len(self._pending))
            return [self._pending.popleft() for _ in range(count)]

    def complete(self, command: SimulationOperatorCommand, *, ok: bool, reason: str) -> None:
        with self._lock:
            for request_id in command.request_ids:
                self._results.append(
                    PendingSimulationResult(
                        target_id=command.ui_id,
                        payload=SimulationResultPayload(
                            request_id=request_id,
                            session_id=command.session_id,
                            command=command.command,
                            ok=bool(ok),
                            reason=str(reason),
                        ),
                    )
                )

    def take_results(self) -> list[PendingSimulationResult]:
        with self._lock:
            results = list(self._results)
            self._results.clear()
            return results


class SimulationOperatorController:
    """Deterministic simulation state machine; all callbacks run on Genesis thread."""

    def __init__(
        self,
        *,
        reset_environment: Callable[[], None],
        observer_command: Callable[[str, dict[str, Any]], None],
    ) -> None:
        self.reset_environment = reset_environment
        self.observer_command = observer_command
        self.paused = False
        self.speed = 1.0
        self.debug_visible = True
        self.epoch = 0
        self._steps_remaining = 0
        self._observer_dirty = False

    def apply(self, command: SimulationOperatorCommand) -> tuple[bool, str]:
        name = command.command
        if name in _CAMERA_COMMANDS:
            self.observer_command(name, dict(command.arguments))
            self._observer_dirty = True
            return True, "view updated"
        if name == "pause":
            self.paused = True
            return True, "paused"
        if name == "resume":
            self.paused = False
            self._steps_remaining = 0
            return True, "running"
        if name == "step":
            if not self.paused:
                return False, "single-step requires a paused simulation"
            self._steps_remaining += int(command.arguments["count"])
            return True, "step queued"
        if name == "reset":
            self.reset()
            return True, "reset"
        if name == "set_speed":
            self.speed = float(command.arguments["scale"])
            return True, "speed updated"
        if name == "set_debug_visible":
            self.debug_visible = bool(command.arguments["visible"])
            return True, "debug visibility updated"
        return False, "unsupported simulation command"

    def reset(self) -> None:
        """Reset the environment while preserving running/paused state."""

        self.reset_environment()
        self.epoch += 1
        self._steps_remaining = 0
        self._observer_dirty = True

    def should_step(self) -> bool:
        if not self.paused:
            return True
        if self._steps_remaining <= 0:
            return False
        self._steps_remaining -= 1
        return True

    def take_observer_dirty(self) -> bool:
        dirty = self._observer_dirty
        self._observer_dirty = False
        return dirty

    def status(self, *, sim_time_s: float) -> SimulationStatusPayload:
        return SimulationStatusPayload(
            epoch=self.epoch,
            paused=self.paused,
            speed=self.speed,
            debug_visible=self.debug_visible,
            sim_time_s=max(0.0, float(sim_time_s)),
        )


__all__ = [
    "PendingSimulationResult",
    "SimulationOperatorCommand",
    "SimulationOperatorController",
    "SimulationOperatorMailbox",
]
