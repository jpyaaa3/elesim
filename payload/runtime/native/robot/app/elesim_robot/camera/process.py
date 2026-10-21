"""Keep vendor SDK/GIL stalls outside the Robot motion-control process."""
from __future__ import annotations

import multiprocessing as mp
import time
from queue import Empty, Full
from typing import Any


def _put_status(queue, status):
    status = dict(status, error=str(status.get("error", ""))[:512])
    try:
        queue.put_nowait(status)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            return
        try:
            queue.put_nowait(status)
        except Full:
            pass


def _run_camera(stop, queue, kwargs):
    worker = None
    backend = kwargs.pop("backend")
    try:
        from .worker import CameraPublisherThread
        factory = kwargs.pop("camera_factory", None)
        if factory is not None:
            pass
        elif backend == "zed":
            from .zed import ZedMiniCamera
            factory = ZedMiniCamera
        else:
            from .realsense import RealSenseCamera
            factory = RealSenseCamera
        worker = CameraPublisherThread(**kwargs, camera_factory=factory)
        worker.start()
        while not stop.value:
            status = worker.status()
            _put_status(queue, dict(status, backend=backend))
            if not status["running"]:
                break
            time.sleep(0.05)
    except Exception as exc:
        _put_status(queue, {"running": False, "healthy": False, "error": str(exc), "backend": backend})
    finally:
        if worker is not None:
            worker.stop()
            _put_status(queue, dict(worker.status(), backend=backend))


class CameraPublisherProcess:
    def __init__(self, topic: str, *, backend: str, **kwargs: Any) -> None:
        if backend not in {"zed", "realsense"}:
            raise ValueError(f"unsupported camera backend: {backend}")
        self._kwargs = dict(kwargs, topic=topic, backend=backend)
        self._context = mp.get_context("spawn")
        # A multiprocessing.Event can itself block its parent if the child
        # holds its condition lock when a vendor call captures the GIL.
        self._stop = self._context.Value("b", False, lock=False)
        self._queue = self._context.Queue(maxsize=1)
        self._process = None
        self._status = {"backend": backend, "running": False, "healthy": False,
                        "error": "", "frames_published": 0, "last_frame_at": 0.0}

    def start(self) -> None:
        if self._process is not None and self._process.is_alive():
            return
        self._stop.value = False
        self._status.update(running=True, healthy=False, error="")
        self._process = self._context.Process(
            target=_run_camera, args=(self._stop, self._queue, dict(self._kwargs)),
            name="robot-rgbd", daemon=True,
        )
        self._process.start()

    def status(self) -> dict[str, object]:
        for _ in range(4):
            try:
                self._status.update(self._queue.get_nowait())
            except Empty:
                break
        if self._process is not None and not self._process.is_alive():
            self._status.update(running=False, healthy=False)
            if not self._status["error"]:
                self._status["error"] = f"camera process exited ({self._process.exitcode})"
        self._status["healthy"] = bool(
            self._status["running"] and not self._status["error"]
            and self._status["frames_published"] > 0
        )
        return dict(self._status)

    def stop(self) -> None:
        self._stop.value = True
        if self._process is not None:
            self._process.join(timeout=3.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
        self._status.update(running=False, healthy=False)
