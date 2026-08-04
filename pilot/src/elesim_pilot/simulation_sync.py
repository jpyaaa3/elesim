"""Synchronize long-running visual workflows with authoritative sim state."""

from __future__ import annotations

import queue
import threading
from typing import Any, Optional

from elesim_protocol import SimulationStatusPayload


def cancellation_reason(
    previous: Optional[SimulationStatusPayload],
    current: SimulationStatusPayload,
) -> str:
    reasons: list[str] = []
    if current.paused and (previous is None or not previous.paused):
        reasons.append("simulation paused")
    if previous is not None and current.epoch != previous.epoch:
        reasons.append(f"simulation epoch changed {previous.epoch}->{current.epoch}")
    return "; ".join(reasons)


class SimulationWorkflowSync:
    """Cancel pilot workflows away from the DDS receive thread."""

    def __init__(self, service: Any, *, autostart: bool = True) -> None:
        self.service = service
        self._lock = threading.Lock()
        self._latest: Optional[SimulationStatusPayload] = None
        self._pending = False
        self._reasons: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        if autostart:
            self.start()

    @property
    def latest(self) -> Optional[SimulationStatusPayload]:
        with self._lock:
            return self._latest

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="pilot-simulation-sync",
            daemon=True,
        )
        self._thread.start()

    def accept(self, status: SimulationStatusPayload) -> None:
        if not isinstance(status, SimulationStatusPayload):
            raise TypeError("simulation sync requires SimulationStatusPayload")
        with self._lock:
            reason = cancellation_reason(self._latest, status)
            self._latest = status
            if not reason or self._pending:
                return
            self._pending = True
        self._reasons.put(reason)

    def process_one(self, *, timeout_s: float = 0.0) -> bool:
        try:
            reason = self._reasons.get(timeout=max(0.0, float(timeout_s)))
        except queue.Empty:
            return False
        try:
            self.service.stop_pick_e2e()
            self.service.stop_gaze_stabilizer()
            print(f"[pilot_agent] visual workflows stopped: {reason}")
        finally:
            with self._lock:
                self._pending = False
        return True

    def _run(self) -> None:
        while not self._stop.is_set():
            self.process_one(timeout_s=0.1)

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3.0)


__all__ = ["SimulationWorkflowSync", "cancellation_reason"]
