from __future__ import annotations

import json
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import zmq
from engine.observability.tracing import sampled_traced


@dataclass(frozen=True)
class PreviewFrame:
    image_bgr: np.ndarray
    meta: dict[str, Any]


class PreviewFramePublisher:
    """Thread-owned ZMQ PUB for latest JPEG preview frames."""

    def __init__(self, endpoint: str, *, jpeg_quality: int = 75) -> None:
        self.endpoint = str(endpoint).strip()
        self.jpeg_quality = int(max(1, min(100, int(jpeg_quality))))
        self._queue: queue.Queue[PreviewFrame] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.published = 0
        self.dropped = 0

    def start(self) -> None:
        if not self.endpoint:
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="preview-publisher", daemon=True)
        self._thread.start()

    def publish(self, image_bgr: np.ndarray, *, meta: Optional[dict[str, Any]] = None) -> None:
        if not self.endpoint:
            return
        self.start()
        frame = PreviewFrame(
            image_bgr=np.ascontiguousarray(image_bgr, dtype=np.uint8).copy(),
            meta=dict(meta or {}),
        )
        while True:
            try:
                self._queue.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    return

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._thread = None

    def _run(self) -> None:
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.PUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SNDHWM, 1)
        try:
            sock.bind(self.endpoint)
            print(f"[preview_stream] publisher bound {self.endpoint}")
            while not self._stop.is_set():
                try:
                    frame = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                self._send_frame(sock, frame)
        except Exception as exc:
            print(f"[preview_stream] publisher failed: {exc}")
        finally:
            try:
                sock.close(0)
            except Exception:
                pass

    @sampled_traced("camera.preview.publish", sample_key="camera.preview.publish", every=60, kind="producer")
    def _send_frame(self, sock: Any, frame: PreviewFrame) -> None:
        try:
            import cv2

            ok, buf = cv2.imencode(
                ".jpg",
                frame.image_bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
            )
            if not ok:
                self.dropped += 1
                return
            h, w = frame.image_bgr.shape[:2]
            meta = {
                "ts": time.time(),
                "width": int(w),
                "height": int(h),
                "encoding": "jpg",
                **dict(frame.meta),
            }
            sock.send_multipart(
                [json.dumps(meta, separators=(",", ":")).encode("utf-8"), buf.tobytes()],
                flags=zmq.NOBLOCK,
            )
            self.published += 1
        except zmq.Again:
            self.dropped += 1
        except Exception:
            self.dropped += 1


class PreviewFrameSubscriber:
    """Latest-frame-only SUB for remote perception preview JPEGs."""

    def __init__(self, endpoint: str) -> None:
        self.endpoint = str(endpoint).strip()
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.RCVHWM, 2)
        self._connected = False

    def connect(self) -> None:
        if self._connected:
            return
        self._sock.connect(self.endpoint)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._connected = True

    def close(self) -> None:
        try:
            self._sock.close(0)
        except Exception:
            pass
        self._connected = False

    @sampled_traced("camera.preview.receive", sample_key="camera.preview.receive", every=60, kind="consumer")
    def recv_latest(self, *, timeout_ms: int = 250) -> Optional[PreviewFrame]:
        if not self.endpoint:
            return None
        if not self._connected:
            self.connect()
        poller = zmq.Poller()
        poller.register(self._sock, zmq.POLLIN)
        try:
            events = dict(poller.poll(timeout=int(timeout_ms)))
        except zmq.ZMQError:
            return None
        if self._sock not in events:
            return None
        latest_parts: Optional[list[bytes]] = None
        while True:
            try:
                latest_parts = self._sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except zmq.ZMQError:
                return None
        if latest_parts is None or len(latest_parts) < 2:
            return None
        try:
            meta = json.loads(latest_parts[0].decode("utf-8"))
        except Exception:
            meta = {}
        try:
            import cv2

            arr = np.frombuffer(latest_parts[1], dtype=np.uint8)
            image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if image is None:
                return None
            return PreviewFrame(image_bgr=np.ascontiguousarray(image, dtype=np.uint8), meta=dict(meta))
        except Exception:
            return None
