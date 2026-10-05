"""Bounded simulation commands, retaining the lease captured at submission.

UiSimSession holds its lock around this queue. Transport sends happen outside
that lock, so taking a command reserves an in-flight slot until result/retry.
"""

from __future__ import annotations

from collections import deque
from dataclasses import replace

from elesim_protocol import SimulationCommandRequest


COALESCED_COMMANDS = frozenset({"orbit", "pan", "zoom", "set_speed", "set_debug_visible"})


def _coalesce(previous: SimulationCommandRequest, current: SimulationCommandRequest) -> SimulationCommandRequest:
    if current.command in {"orbit", "pan"}:
        arguments = {
            axis: max(-2.0, min(2.0, float(previous.arguments[axis]) + float(current.arguments[axis])))
            for axis in ("dx", "dy")
        }
        return replace(current, arguments=arguments)
    if current.command == "zoom":
        delta = float(previous.arguments["delta"]) + float(current.arguments["delta"])
        return replace(current, arguments={"delta": max(-2.0, min(2.0, delta))})
    return current


class SimulationCommandQueue:
    def __init__(self, capacity: int) -> None:
        self.capacity = capacity
        self._queued: deque[SimulationCommandRequest] = deque()
        self._inflight: set[str] = set()

    def __bool__(self) -> bool:
        return bool(self._queued)

    @property
    def pending(self) -> int:
        return len(self._queued) + len(self._inflight)

    def enqueue(self, request: SimulationCommandRequest) -> bool:
        if (
            request.command in COALESCED_COMMANDS and self._queued
            and self._queued[-1].command == request.command
            and self._queued[-1].session_id == request.session_id
        ):
            self._queued[-1] = _coalesce(self._queued[-1], request)
            return True
        if self.pending >= self.capacity:
            return False
        self._queued.append(request)
        return True

    def take(self) -> SimulationCommandRequest:
        request = self._queued.popleft()
        self._inflight.add(request.request_id)
        return request

    def retry(self, request: SimulationCommandRequest) -> None:
        if request.request_id in self._inflight:
            self._inflight.remove(request.request_id)
            self._queued.appendleft(request)

    def complete(self, request_id: str) -> None:
        self._inflight.discard(request_id)

    def clear(self) -> None:
        self._queued.clear()
        self._inflight.clear()
