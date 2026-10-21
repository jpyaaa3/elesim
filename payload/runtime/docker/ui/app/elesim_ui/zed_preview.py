"""Latest-only ZED color preview received from the Pilot RGB-D broker."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

from elesim_protocol import (
    DdsEncodedRgbdSubscriber,
    DdsRuntimeSettings,
    decode_image_payload,
)


def _decode_color(frame: Any) -> Any:
    """Decode only color for display; depth remains available to Pilot."""

    codec = str(frame.color_codec).strip().lower()
    if codec in {"jpeg", "png"}:
        import av

        decoder = av.CodecContext.create("mjpeg" if codec == "jpeg" else "png", "r")
        decoded = decoder.decode(av.Packet(bytes(frame.color_payload)))
        if not decoded:
            raise ValueError(f"PyAV returned no frame for {codec} ZED image")
        return decoded[0].to_ndarray(format="bgr24")

    image = decode_image_payload(
        frame.color_payload,
        codec,
        width=int(frame.width),
        height=int(frame.height),
        encoding=str(frame.color_encoding),
    )
    if str(frame.color_encoding).strip().lower() == "rgb8":
        return image[..., ::-1].copy()
    return image


class ZedCameraPreview:
    """Receive the configured Pilot-broker topic off the UI render thread."""

    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str,
        settings: DdsRuntimeSettings,
        stream_id: str = "zed-preview",
        subscriber_factory: Callable[..., Any] = DdsEncodedRgbdSubscriber,
        decoder: Callable[[Any], Any] = _decode_color,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.topic = str(topic).strip()
        if not self.topic.startswith("/"):
            raise ValueError("ZED preview topic must be an absolute DDS topic")
        self._subscriber_factory = subscriber_factory
        self._endpoint_id = f"{endpoint_id}-{str(stream_id).strip() or 'zed-preview'}"
        self._settings = settings
        self._subscriber: Any = None
        self._decoder = decoder
        self._clock = clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame: Any = None
        self._source_id = ""
        self._version = 0
        self._last_frame_at = 0.0
        self._last_error = ""

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="elesim-ui-zed-preview",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        subscriber = self._subscriber
        if subscriber is not None:
            subscriber.close()

    @property
    def frame(self) -> Any:
        with self._lock:
            return self._frame

    @property
    def frame_version(self) -> int:
        with self._lock:
            return self._version

    @property
    def source_id(self) -> str:
        with self._lock:
            return self._source_id

    @property
    def status(self) -> str:
        with self._lock:
            frame_at = self._last_frame_at
            has_frame = self._frame is not None
            error = self._last_error
        if not has_frame:
            return "ERROR" if error else "WAIT"
        return "LIVE" if self._clock() - frame_at <= 2.0 else "STALE"

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                if self._subscriber is None:
                    self._subscriber = self._subscriber_factory(
                        self.topic,
                        endpoint_id=self._endpoint_id,
                        settings=self._settings,
                    )
                    self._subscriber.connect()
                encoded = self._subscriber.recv_latest(timeout_ms=100)
            except Exception as exc:
                self._set_error(exc)
                subscriber, self._subscriber = self._subscriber, None
                if subscriber is not None:
                    try:
                        subscriber.close()
                    except Exception:
                        pass
                self._stop.wait(0.25)
                continue
            if encoded is None:
                continue
            try:
                color = self._decoder(encoded)
            except Exception as exc:
                self._set_error(exc)
                continue
            with self._lock:
                self._frame = color
                self._source_id = str(getattr(encoded, "source_id", "")).strip()
                self._version += 1
                self._last_frame_at = self._clock()
                self._last_error = ""

    def _set_error(self, error: Exception) -> None:
        with self._lock:
            self._last_error = f"{type(error).__name__}: {error}"[:240]


__all__ = ["ZedCameraPreview"]
