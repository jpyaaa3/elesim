"""Latest-only frame publication workers used by the Sim process.

The Genesis scene owns camera rendering and therefore remains on the scene's
runtime thread.  Once a frame has been rendered, however, publishing it to
DDS, copying it into the WebRTC mailbox, and updating the in-process frame hub
are transport operations.  This worker gives those operations a bounded
latest-only handoff so a slow publisher cannot make the physics loop wait.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Iterable
from typing import Any, Optional


class FrameDispatchWorker:
    """Dispatch at most one pending frame per named stream.

    ``submit`` only replaces a reference under a short lock and signals the
    worker.  There is no queue: if the consumer is busy, older frames are
    overwritten.  The worker is intentionally generic so it can publish to
    the local RGB-D publisher and to the media mailbox without importing
    Genesis or DDS in this module.
    """

    def __init__(
        self,
        streams: Iterable[str],
        handler: Callable[[str, Any], None],
        *,
        name: str = "sim-frame-dispatch",
        on_error: Optional[Callable[[str, Exception], None]] = None,
    ) -> None:
        names = tuple(str(stream).strip() for stream in streams)
        if not names or any(not name for name in names) or len(set(names)) != len(names):
            raise ValueError("frame dispatch streams must be non-empty and unique")
        if not callable(handler):
            raise TypeError("frame dispatch handler must be callable")
        self._streams = names
        self._handler = handler
        self._on_error = on_error
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stop = threading.Event()
        self._slots: dict[str, Any] = {name: None for name in names}
        self._submitted = {name: 0 for name in names}
        self._processed = {name: 0 for name in names}
        self._overwritten = {name: 0 for name in names}
        self._failed = {name: 0 for name in names}
        self._last_error: dict[str, str] = {name: "" for name in names}
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._started = False
        self._closed = False

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def start(self) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError("frame dispatch worker is closed")
            if self._started:
                return
            self._started = True
        self._thread.start()

    def submit(self, stream: str, frame: Any) -> bool:
        name = str(stream)
        with self._lock:
            if name not in self._slots:
                raise KeyError(f"unknown frame stream: {name}")
            if self._closed or self._stop.is_set():
                return False
            if self._slots[name] is not None:
                self._overwritten[name] += 1
            self._slots[name] = frame
            self._submitted[name] += 1
            self._event.set()
            return True

    def stats(self) -> dict[str, dict[str, int | str]]:
        with self._lock:
            return {
                name: {
                    "submitted": self._submitted[name],
                    "processed": self._processed[name],
                    "overwritten": self._overwritten[name],
                    "failed": self._failed[name],
                    "last_error": self._last_error[name],
                }
                for name in self._streams
            }

    def flush(self, stream: str, *, timeout_s: float = 1.0) -> bool:
        """Wait until the frames submitted so far for ``stream`` are handled.

        Camera capture remains on the Genesis thread, but the first frame must
        reach the shared media mailbox before the simulation-session readiness
        gate is advertised.  This is a bounded barrier for that startup edge;
        normal steady-state publication remains latest-only and asynchronous.
        """

        name = str(stream)
        with self._lock:
            if name not in self._slots:
                raise KeyError(f"unknown frame stream: {name}")
            target = int(self._submitted[name])
            failed_before = int(self._failed[name])
            handled = int(self._processed[name]) + int(self._overwritten[name])
            if target <= handled:
                return (
                    target > 0
                    and not self._closed
                    and not self._stop.is_set()
                    and int(self._failed[name]) == failed_before
                )
        deadline = time.monotonic() + max(0.01, float(timeout_s))
        while time.monotonic() < deadline:
            with self._lock:
                handled = int(self._processed[name]) + int(self._overwritten[name])
                if handled >= target:
                    return int(self._failed[name]) == failed_before
                if self._closed or self._stop.is_set():
                    return False
            self._event.set()
            time.sleep(0.002)
        with self._lock:
            handled = int(self._processed[name]) + int(self._overwritten[name])
            return (
                handled >= target
                and int(self._failed[name]) == failed_before
            )

    def close(self, *, timeout_s: float = 2.0) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._slots = {name: None for name in self._streams}
        self._stop.set()
        self._event.set()
        if self._started:
            self._thread.join(timeout=max(0.1, float(timeout_s)))

    def _take_batch(self) -> list[tuple[str, Any]]:
        with self._lock:
            batch = [
                (name, frame)
                for name, frame in self._slots.items()
                if frame is not None
            ]
            for name, _frame in batch:
                self._slots[name] = None
            if not batch:
                self._event.clear()
            return batch

    def _run(self) -> None:
        while not self._stop.is_set():
            self._event.wait(0.1)
            while not self._stop.is_set():
                batch = self._take_batch()
                if not batch:
                    break
                for name, frame in batch:
                    try:
                        self._handler(name, frame)
                    except Exception as exc:  # pragma: no cover - callback specific
                        detail = str(exc).replace("\n", " ").strip()[:512]
                        with self._lock:
                            self._failed[name] += 1
                            self._last_error[name] = detail or type(exc).__name__
                        if self._on_error is not None:
                            try:
                                self._on_error(name, exc)
                            except Exception:
                                pass
                    finally:
                        with self._lock:
                            self._processed[name] += 1


__all__ = ["FrameDispatchWorker"]
