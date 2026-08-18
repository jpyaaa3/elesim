"""Bounded Sim media worker and latest-only frame mailboxes.

Genesis owns the simulation state and DDS authority.  This module moves only
WebRTC peer connections and video encoding behind a private process boundary.
The boundary has exactly one frame slot per stream and a bounded request pipe;
there is intentionally no queue which can accumulate stale video frames.
"""

from __future__ import annotations

import ctypes
import multiprocessing as mp
import threading
import time
from dataclasses import dataclass
from multiprocessing.connection import Connection
from typing import Any, Mapping, Optional

import numpy as np


class MediaWorkerError(RuntimeError):
    """The media worker rejected or failed an operation."""


class MediaWorkerUnavailable(MediaWorkerError):
    """The media worker is not alive or has not completed its handshake."""


@dataclass(frozen=True)
class VideoStreamSpec:
    """Fixed media parameters shared by the producer and WebRTC worker."""

    name: str
    fps: float
    width: int
    height: int

    def __post_init__(self) -> None:
        name = str(self.name).strip()
        if not name:
            raise ValueError("video stream name is required")
        fps = float(self.fps)
        width = int(self.width)
        height = int(self.height)
        if fps <= 0.0 or not np.isfinite(fps):
            raise ValueError("video stream fps must be finite and positive")
        if width <= 0 or height <= 0:
            raise ValueError("video stream dimensions must be positive")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "fps", fps)
        object.__setattr__(self, "width", width)
        object.__setattr__(self, "height", height)


class SharedFrameMailbox:
    """One bounded latest-only BGR frame slot shared with a child process.

    ``publish`` never waits for an encoder queue.  It replaces the previous
    frame in a fixed-size shared buffer and returns a monotonically increasing
    sequence.  The lock covers only the bounded memory copy; the worker copies
    to a private NumPy array before doing any WebRTC/codec work.
    """

    def __init__(
        self,
        *,
        width: int,
        height: int,
        buffer: Any,
        lock: Any,
        sequence: Any,
        captured_at: Any,
        published: Any,
        overwritten: Any,
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self._buffer = buffer
        self._lock = lock
        self._sequence = sequence
        self._captured_at = captured_at
        self._published = published
        self._overwritten = overwritten

    @classmethod
    def create(cls, context: mp.context.BaseContext, *, width: int, height: int) -> "SharedFrameMailbox":
        width_i = int(width)
        height_i = int(height)
        if width_i <= 0 or height_i <= 0:
            raise ValueError("mailbox dimensions must be positive")
        return cls(
            width=width_i,
            height=height_i,
            buffer=context.RawArray(ctypes.c_ubyte, width_i * height_i * 3),
            lock=context.Lock(),
            sequence=context.Value(ctypes.c_ulonglong, 0),
            captured_at=context.Value(ctypes.c_double, 0.0),
            published=context.Value(ctypes.c_ulonglong, 0),
            overwritten=context.Value(ctypes.c_ulonglong, 0),
        )

    @property
    def shape(self) -> tuple[int, int, int]:
        return self.height, self.width, 3

    def publish(self, frame: np.ndarray, *, captured_at: Optional[float] = None) -> int:
        image = np.asarray(frame)
        if image.dtype != np.uint8 or image.shape != self.shape:
            raise ValueError(
                f"mailbox frame must be uint8 {self.shape}, got "
                f"{image.dtype} {tuple(image.shape)}"
            )
        contiguous = np.ascontiguousarray(image)
        now = time.monotonic() if captured_at is None else float(captured_at)
        with self._lock:
            if self._sequence.value:
                self._overwritten.value += 1
            target = np.frombuffer(self._buffer, dtype=np.uint8).reshape(self.shape)
            np.copyto(target, contiguous, casting="no")
            self._sequence.value += 1
            self._captured_at.value = now
            self._published.value += 1
            return int(self._sequence.value)

    def latest(self) -> tuple[Optional[np.ndarray], int, float]:
        """Return a private copy, sequence and capture timestamp."""

        with self._lock:
            sequence = int(self._sequence.value)
            captured_at = float(self._captured_at.value)
            if sequence <= 0:
                return None, 0, captured_at
            source = np.frombuffer(self._buffer, dtype=np.uint8).reshape(self.shape)
            image = source.copy()
        return image, sequence, captured_at

    def stats(self) -> dict[str, int | float]:
        with self._lock:
            return {
                "sequence": int(self._sequence.value),
                "captured_at": float(self._captured_at.value),
                "published": int(self._published.value),
                "overwritten": int(self._overwritten.value),
            }


def _worker_send(connection: Connection, message: Mapping[str, Any]) -> bool:
    try:
        connection.send(dict(message))
        return True
    except (BrokenPipeError, EOFError, OSError):
        return False


def _media_worker_main(
    connection: Connection,
    specs: tuple[VideoStreamSpec, ...],
    mailboxes: Mapping[str, SharedFrameMailbox],
) -> None:
    """Child entrypoint; importing Genesis is intentionally impossible here."""

    sender = None
    try:
        from elesim_sim.vision.webrtc import (
            NamedWebRtcVideoSender,
            available,
            configure_h264_encoder,
            h264_rtp_payload_max,
        )

        if not available():
            _worker_send(connection, {"ok": False, "error": "aiortc/av unavailable"})
            return

        encoder = configure_h264_encoder()
        print(
            f"[sim-media] h264 encoder={encoder} "
            f"rtp_payload_max={h264_rtp_payload_max()}",
            flush=True,
        )

        providers = {
            spec.name: (lambda mailbox=mailboxes[spec.name]: mailbox.latest()[0])
            for spec in specs
        }
        sender = NamedWebRtcVideoSender(
            providers,
            fps={spec.name: spec.fps for spec in specs},
            frame_sizes={spec.name: (spec.width, spec.height) for spec in specs},
        )
        if not _worker_send(connection, {"ok": True, "event": "ready"}):
            return

        while True:
            if not connection.poll(0.1):
                continue
            try:
                command = connection.recv()
            except (EOFError, OSError):
                break
            if not isinstance(command, tuple) or not command:
                _worker_send(connection, {"ok": False, "error": "invalid media command"})
                continue
            name = str(command[0])
            if name == "shutdown":
                _worker_send(connection, {"ok": True, "event": "stopped"})
                break
            if name == "accept_offer" and len(command) == 6:
                _, stream, sdp, offer_type, turn, session_id = command
                try:
                    answer = sender.accept_offer(
                        str(stream),
                        str(sdp),
                        str(offer_type),
                        turn,
                        str(session_id),
                    )
                    _worker_send(connection, {"ok": True, "answer": dict(answer)})
                except Exception as exc:  # pragma: no cover - codec/ICE specific
                    _worker_send(
                        connection,
                        {"ok": False, "error": _bounded_error("accept_offer", exc)},
                    )
                continue
            if name == "close_session" and len(command) == 2:
                try:
                    sender.close_session(str(command[1]))
                    _worker_send(connection, {"ok": True})
                except Exception as exc:  # pragma: no cover - codec specific
                    _worker_send(
                        connection,
                        {"ok": False, "error": _bounded_error("close_session", exc)},
                    )
                continue
            _worker_send(connection, {"ok": False, "error": "invalid media command"})
    except BaseException as exc:  # pragma: no cover - process boundary guard
        _worker_send(connection, {"ok": False, "error": _bounded_error("startup", exc)})
    finally:
        if sender is not None:
            try:
                sender.close()
            except Exception:
                pass
        try:
            connection.close()
        except Exception:
            pass


def _bounded_error(stage: str, exc: BaseException) -> str:
    detail = str(exc).replace("\n", " ").replace("\r", " ").strip()
    return f"{stage}: {detail or exc.__class__.__name__}"[:512]


class MediaWorkerClient:
    """Parent-side bounded proxy for the private WebRTC process."""

    def __init__(
        self,
        specs: Mapping[str, VideoStreamSpec],
        *,
        context: Optional[mp.context.BaseContext] = None,
        command_timeout_s: float = 15.0,
    ) -> None:
        normalized = tuple(specs.values())
        if not normalized:
            raise ValueError("media worker requires at least one stream")
        if len({spec.name for spec in normalized}) != len(normalized):
            raise ValueError("media worker stream names must be unique")
        self.specs = normalized
        self.context = context or mp.get_context("spawn")
        self.mailboxes = {
            spec.name: SharedFrameMailbox.create(
                self.context,
                width=spec.width,
                height=spec.height,
            )
            for spec in normalized
        }
        self._parent, child = self.context.Pipe(duplex=True)
        self._child = child
        self._process = self.context.Process(
            target=_media_worker_main,
            args=(child, self.specs, self.mailboxes),
            name="elesim-sim-media",
            daemon=True,
        )
        self._command_lock = threading.Lock()
        self._command_timeout_s = max(1.0, float(command_timeout_s))
        self._started = False
        self._ready = threading.Event()
        self._failure = ""
        self._closing = False

    @property
    def process(self) -> mp.Process:
        return self._process

    @property
    def ready(self) -> bool:
        return self._ready.is_set() and self._process.is_alive()

    @property
    def failure(self) -> str:
        return self._failure

    def start(self, *, timeout_s: float = 8.0) -> None:
        self._closing = False
        if self._started:
            if not self.ready:
                raise MediaWorkerUnavailable(self._failure or "media worker is not ready")
            return
        self._process.start()
        try:
            self._child.close()
        except Exception:
            pass
        self._started = True
        try:
            response = self._wait_response(timeout_s)
        except Exception:
            self.close()
            raise
        if not response.get("ok") or response.get("event") != "ready":
            self._failure = str(response.get("error") or "media worker startup failed")
            self.close()
            raise MediaWorkerUnavailable(self._failure)
        self._ready.set()
        print(
            "[sim-media] worker ready streams="
            + ",".join(spec.name for spec in self.specs),
            flush=True,
        )

    def accept_offer(
        self,
        stream: str,
        sdp: str,
        offer_type: str,
        turn: Any,
        session_id: str,
    ) -> dict[str, str]:
        response = self._request(
            ("accept_offer", str(stream), str(sdp), str(offer_type), turn, str(session_id)),
        )
        answer = response.get("answer")
        if not isinstance(answer, dict):
            raise MediaWorkerError("media worker returned an invalid WebRTC answer")
        return {"sdp": str(answer.get("sdp", "")), "type": str(answer.get("type", ""))}

    def close_session(self, session_id: str) -> None:
        self._request(("close_session", str(session_id)))

    def diagnostics(self) -> dict[str, Any]:
        return {
            "ready": self.ready,
            "alive": bool(self._process.is_alive()) if self._started else False,
            "failure": self._failure,
            "streams": {
                name: mailbox.stats() for name, mailbox in self.mailboxes.items()
            },
        }

    def _request(self, command: tuple[Any, ...]) -> dict[str, Any]:
        if self._closing or not self.ready:
            raise MediaWorkerUnavailable(self._failure or "media worker is not ready")
        with self._command_lock:
            try:
                self._parent.send(command)
            except (BrokenPipeError, EOFError, OSError) as exc:
                self._failure = "media worker pipe closed"
                self._ready.clear()
                raise MediaWorkerUnavailable(self._failure) from exc
            response = self._wait_response(self._command_timeout_s)
        if not response.get("ok"):
            raise MediaWorkerError(str(response.get("error") or "media worker request failed"))
        return response

    def _wait_response(self, timeout_s: float) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self._failure = "media worker response timed out"
                self._ready.clear()
                raise MediaWorkerUnavailable(self._failure)
            try:
                if not self._parent.poll(min(0.1, remaining)):
                    if self._started and not self._process.is_alive():
                        self._failure = "media worker exited unexpectedly"
                        self._ready.clear()
                        raise MediaWorkerUnavailable(self._failure)
                    continue
                response = self._parent.recv()
            except (EOFError, OSError) as exc:
                self._failure = "media worker pipe closed"
                self._ready.clear()
                raise MediaWorkerUnavailable(self._failure) from exc
            if isinstance(response, dict):
                return response
            self._failure = "media worker returned an invalid response"
            self._ready.clear()
            raise MediaWorkerUnavailable(self._failure)

    def close(self) -> None:
        if not self._started:
            return
        self._closing = True
        self._ready.clear()
        if self._process.is_alive():
            acquired = self._command_lock.acquire(timeout=0.5)
            if acquired:
                try:
                    self._parent.send(("shutdown",))
                    self._wait_response(2.0)
                except Exception:
                    pass
                finally:
                    self._command_lock.release()
        if self._process.is_alive():
            self._process.terminate()
        self._process.join(timeout=3.0)
        try:
            self._parent.close()
        except Exception:
            pass
        try:
            self._child.close()
        except Exception:
            pass
        self._started = False


__all__ = [
    "MediaWorkerClient",
    "MediaWorkerError",
    "MediaWorkerUnavailable",
    "SharedFrameMailbox",
    "VideoStreamSpec",
]
