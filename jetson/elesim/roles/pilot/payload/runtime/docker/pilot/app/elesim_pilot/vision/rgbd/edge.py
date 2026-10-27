"""Pilot-side RGB-D edge processing.

The edge is separate from DDS. Pilot owns one bounded latest-only slot,
performs one encoding operation, and hands an immutable protocol
``EncodedRgbdFrame`` to a future DDS publisher. Already encoded samples pass
through without a decode/re-encode cycle.
"""

from __future__ import annotations

import enum
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, Union

import numpy as np

from elesim_protocol.encoded_rgbd import (
    EncodedRgbdFrame,
    RgbdEncodedMetadata,
    encode_image_payload,
    validate_encoded_frame,
)

from .types import RgbdFrame


class RgbdEdgeError(ValueError):
    """Raised when an RGB-D source cannot be encoded safely."""


class RgbdCodec(str, enum.Enum):
    RAW = "raw"
    ZLIB = "zlib"
    JPEG = "jpeg"
    PNG = "png"


@dataclass(frozen=True)
class RgbdEncodingPolicy:
    """Encoding choices for the Pilot edge.

    Color is JPEG-compressed once at the edge to avoid shipping raw camera
    bytes over DDS. Depth remains lossless and uses zlib by default because it
    is consumed by perception. The codec metadata leaves room for a hardware
    encoder without changing the broker API.
    """

    color: RgbdCodec | str = RgbdCodec.JPEG
    depth: RgbdCodec | str = RgbdCodec.ZLIB
    zlib_level: int = 1
    jpeg_quality: int = 85

    def __post_init__(self) -> None:
        _codec(self.color, "color")
        _codec(self.depth, "depth")
        if not 0 <= int(self.zlib_level) <= 9:
            raise RgbdEdgeError("RGB-D zlib level must be in [0, 9]")
        if not 1 <= int(self.jpeg_quality) <= 100:
            raise RgbdEdgeError("RGB-D JPEG quality must be in [1, 100]")

    @property
    def color_codec(self) -> RgbdCodec:
        return _codec(self.color, "color")

    @property
    def depth_codec(self) -> RgbdCodec:
        return _codec(self.depth, "depth")


class RgbdEncoder(Protocol):
    def __call__(self, frame: RgbdFrame) -> EncodedRgbdFrame:
        ...


def _codec(value: RgbdCodec | str, label: str) -> RgbdCodec:
    try:
        return value if isinstance(value, RgbdCodec) else RgbdCodec(str(value).lower())
    except ValueError as exc:
        raise RgbdEdgeError(f"unsupported RGB-D {label} codec: {value!r}") from exc


def _vector(value: Any, size: int) -> Optional[tuple[float, ...]]:
    if value is None:
        return None
    result = tuple(float(component) for component in value)
    if len(result) != size:
        raise RgbdEdgeError(f"RGB-D pose metadata must contain {size} values")
    return result


def encode_rgbd_frame(
    frame: RgbdFrame,
    *,
    policy: RgbdEncodingPolicy | None = None,
    source_id: str = "pilot-rgbd",
    source_boot_id: str = "pilot-local",
    calibration_id: str = "",
) -> EncodedRgbdFrame:
    """Encode one raw source frame into the bounded protocol value."""

    selected = policy or RgbdEncodingPolicy()
    try:
        color = np.ascontiguousarray(frame.color_bgr, dtype=np.uint8)
        if color.ndim != 3 or color.shape[2] != 3:
            raise RgbdEdgeError("RGB-D color image must have shape HxWx3")
        depth = np.ascontiguousarray(frame.depth_raw)
        if depth.ndim != 2 or depth.shape != color.shape[:2]:
            raise RgbdEdgeError("RGB-D depth image must match color dimensions")
        if depth.dtype not in (np.dtype(np.uint16), np.dtype(np.float32)):
            depth = np.ascontiguousarray(depth, dtype=np.uint16)
        intrinsics = frame.intrinsics
        encoded = EncodedRgbdFrame(
            source_id=str(source_id),
            source_boot_id=str(source_boot_id),
            seq=int(getattr(frame, "seq", 0)),
            ts=float(getattr(frame, "ts", 0.0)),
            width=int(color.shape[1]),
            height=int(color.shape[0]),
            color_codec=selected.color_codec.value,
            color_encoding="bgr8",
            color_payload=encode_image_payload(
                color,
                selected.color_codec.value,
                encoding="bgr8",
                quality=int(selected.jpeg_quality),
                zlib_level=int(selected.zlib_level),
            ),
            depth_codec=selected.depth_codec.value,
            depth_encoding="16uc1" if depth.dtype == np.uint16 else "32fc1",
            depth_payload=encode_image_payload(
                depth,
                selected.depth_codec.value,
                encoding="16uc1" if depth.dtype == np.uint16 else "32fc1",
                quality=int(selected.jpeg_quality),
                zlib_level=int(selected.zlib_level),
            ),
            depth_scale=float(frame.depth_scale),
            metadata=RgbdEncodedMetadata(
                fx=float(intrinsics.fx),
                fy=float(intrinsics.fy),
                cx=float(intrinsics.cx),
                cy=float(intrinsics.cy),
                calibration_id=str(calibration_id),
                arm_q=_vector(getattr(frame, "arm_q", None), 4),
                camera_world_origin=_vector(getattr(frame, "camera_world_origin", None), 3),
                camera_world_look=_vector(getattr(frame, "camera_world_look", None), 3),
                camera_world_right=_vector(getattr(frame, "camera_world_right", None), 3),
            ),
        )
        return validate_encoded_frame(encoded, decode_payloads=False)
    except RgbdEdgeError:
        raise
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise RgbdEdgeError(f"invalid RGB-D source frame: {exc}") from exc


def encoded_frame_from_message(message: Any) -> EncodedRgbdFrame:
    """Adapt a generated ``elesim_interfaces/EncodedRgbdFrame`` message."""

    source = message.source
    capture_time = message.capture_time
    has_depth = bool(message.has_depth)
    pose = bool(message.has_camera_pose)
    frame = EncodedRgbdFrame(
        source_id=str(source.endpoint_id),
        source_boot_id=str(source.boot_id),
        seq=int(message.frame_sequence),
        ts=float(capture_time.sec) + float(capture_time.nanosec) / 1_000_000_000.0,
        width=int(message.width),
        height=int(message.height),
        color_codec=str(message.color_codec),
        color_encoding=str(message.color_encoding),
        color_payload=bytes(message.color_data),
        depth_codec=str(message.depth_codec) if has_depth else "",
        depth_encoding=str(message.depth_encoding) if has_depth else "",
        depth_payload=bytes(message.depth_data) if has_depth else b"",
        depth_scale=float(message.depth_scale) if has_depth else 0.0,
        metadata=RgbdEncodedMetadata(
            fx=float(message.fx),
            fy=float(message.fy),
            cx=float(message.cx),
            cy=float(message.cy),
            calibration_id=str(message.calibration_id),
            arm_q=tuple(float(v) for v in message.arm_q) if bool(message.has_arm_q) else None,
            camera_world_origin=tuple(float(v) for v in message.camera_world_origin) if pose else None,
            camera_world_look=tuple(float(v) for v in message.camera_world_look) if pose else None,
            camera_world_right=tuple(float(v) for v in message.camera_world_right) if pose else None,
        ),
        has_depth=has_depth,
    )
    return validate_encoded_frame(frame, decode_payloads=False)


class RgbdEdgeStats:
    """A point-in-time snapshot of bounded edge activity."""

    def __init__(self, *, submitted: int, encoded: int, passed_through: int,
                 replaced: int, failed: int, delivered: int) -> None:
        self.submitted = int(submitted)
        self.encoded = int(encoded)
        self.passed_through = int(passed_through)
        self.replaced = int(replaced)
        self.failed = int(failed)
        self.delivered = int(delivered)


class RgbdEdgeBroker:
    """Encode/pass through RGB-D with one bounded latest-only slot."""

    def __init__(
        self,
        *,
        policy: RgbdEncodingPolicy | None = None,
        encoder: RgbdEncoder | None = None,
        on_frame: Optional[Callable[[EncodedRgbdFrame], None]] = None,
        source_id: str = "pilot-rgbd",
        source_boot_id: str = "pilot-local",
        calibration_id: str = "",
        worker: bool = True,
    ) -> None:
        self.policy = policy or RgbdEncodingPolicy()
        self._encoder = encoder or (lambda frame: encode_rgbd_frame(
            frame, policy=self.policy, source_id=source_id,
            source_boot_id=source_boot_id, calibration_id=calibration_id,
        ))
        self._on_frame = on_frame
        self._worker_enabled = bool(worker)
        self._condition = threading.Condition()
        self._pending: Union[RgbdFrame, EncodedRgbdFrame, None] = None
        self._latest: Optional[EncodedRgbdFrame] = None
        self._closed = False
        self._thread: Optional[threading.Thread] = None
        self._submitted = self._encoded = self._passed_through = 0
        self._replaced = self._failed = self._delivered = 0
        self.last_error: Optional[str] = None
        if self._worker_enabled:
            self._thread = threading.Thread(target=self._run, name="elesim-rgbd-edge", daemon=True)
            self._thread.start()

    def submit(self, frame: RgbdFrame | EncodedRgbdFrame) -> bool:
        """Transfer one source slot; return ``False`` after close.

        Raw frame arrays are borrowed until the worker consumes them. This
        avoids a defensive CPU copy; producers must not mutate them meanwhile.
        """

        with self._condition:
            if self._closed:
                return False
            self._submitted += 1
            if self._pending is not None:
                self._replaced += 1
            self._pending = frame
            self._condition.notify()
        if not self._worker_enabled:
            self._process_one()
        return True

    def submit_encoded_message(self, message: Any) -> bool:
        """Submit a generated encoded DDS sample without re-encoding it."""

        return self.submit(encoded_frame_from_message(message))

    def recv_latest(self, *, timeout_s: float = 0.0) -> Optional[EncodedRgbdFrame]:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while self._latest is None and not self._closed and timeout_s > 0:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            result = self._latest
            self._latest = None
            return result

    def stats(self) -> RgbdEdgeStats:
        with self._condition:
            return RgbdEdgeStats(
                submitted=self._submitted, encoded=self._encoded,
                passed_through=self._passed_through, replaced=self._replaced,
                failed=self._failed, delivered=self._delivered,
            )

    def close(self, *, timeout_s: float = 1.0) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._pending = None
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(max(0.0, float(timeout_s)))
            self._thread = None

    def __enter__(self) -> "RgbdEdgeBroker":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._closed and self._pending is None:
                    return
            self._process_one()

    def _process_one(self) -> None:
        with self._condition:
            source = self._pending
            self._pending = None
            if source is None:
                return
        try:
            if isinstance(source, EncodedRgbdFrame):
                encoded = source
                validate_encoded_frame(encoded, decode_payloads=False)
                with self._condition:
                    self._passed_through += 1
            else:
                encoded = self._encoder(source)
                if not isinstance(encoded, EncodedRgbdFrame):
                    raise RgbdEdgeError("RGB-D encoder returned an invalid frame")
                with self._condition:
                    self._encoded += 1
            callback = self._on_frame
            if callback is not None:
                # If a newer source arrived while this frame was being
                # encoded, keep the edge latest-only contract strict: do not
                # publish or expose an already-obsolete frame.  The worker
                # loop immediately consumes the replacement.
                with self._condition:
                    superseded = self._pending is not None and not self._closed
                if superseded:
                    return
                callback(encoded)
            with self._condition:
                if self._pending is not None and not self._closed:
                    # The callback may itself take enough time for a newer
                    # frame to arrive; avoid publishing a stale slot then.
                    return
                self._latest = encoded
                self._delivered += 1
                self.last_error = None
                self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                self._failed += 1
                self.last_error = f"{exc.__class__.__name__}: {exc}"[:512]
                self._condition.notify_all()


class DdsRgbdEdgePublisher:
    """Pilot convenience adapter: raw source -> encoded DDS exactly once.

    The DDS object is created only when this explicit adapter is used; the
    legacy raw subscriber/publisher path is unaffected.  The broker callback
    is the sole call site that publishes encoded samples, keeping the bounded
    latest-only policy visible at the Pilot boundary.
    """

    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str,
        settings: Any = None,
        boot_id: str = "",
        policy: RgbdEncodingPolicy | None = None,
        source_id: str = "pilot-rgbd",
        source_boot_id: str = "pilot-local",
        calibration_id: str = "",
    ) -> None:
        from elesim_protocol.encoded_rgbd import DdsEncodedRgbdPublisher

        self._publisher = DdsEncodedRgbdPublisher(
            topic,
            endpoint_id=endpoint_id,
            settings=settings,
            boot_id=boot_id,
        )
        self.broker = RgbdEdgeBroker(
            policy=policy,
            source_id=source_id,
            source_boot_id=source_boot_id,
            calibration_id=calibration_id,
            on_frame=self._publisher.publish,
        )

    def submit(self, frame: RgbdFrame | EncodedRgbdFrame) -> bool:
        return self.broker.submit(frame)

    def submit_encoded_message(self, message: Any) -> bool:
        return self.broker.submit_encoded_message(message)

    def stats(self) -> RgbdEdgeStats:
        return self.broker.stats()

    def close(self) -> None:
        self.broker.close()
        self._publisher.close()


class DdsRgbdRelay:
    """Pilot-owned relay from a source topic to the encoded broker topic.

    A source may already be encoded (the normal Sim/Robot path) or may be a
    legacy raw ``RgbdFrame`` topic.  In the latter case this class performs the
    one allowed edge encode before publishing.  Both paths keep only a single
    latest sample and never let a DDS receive backlog accumulate.
    """

    def __init__(
        self,
        source_topic: str,
        broker_topic: str,
        *,
        source_format: str,
        endpoint_id: str,
        settings: Any = None,
        expected_source_id: str = "",
        expected_boot_id: str = "",
        policy: RgbdEncodingPolicy | None = None,
    ) -> None:
        from elesim_protocol import DdsEncodedRgbdSubscriber, DdsRgbdSubscriber

        self.source_topic = str(source_topic)
        self.broker_topic = str(broker_topic)
        self.source_format = str(source_format).strip().lower() or "raw-rgbd-v1"
        if self.source_format not in {"raw-rgbd-v1", "encoded-rgbd-v1"}:
            raise RgbdEdgeError(f"unsupported RGB-D source format: {source_format!r}")
        subscriber_type = (
            DdsEncodedRgbdSubscriber
            if self.source_format == "encoded-rgbd-v1"
            else DdsRgbdSubscriber
        )
        self._subscriber = subscriber_type(
            self.source_topic,
            endpoint_id=f"{endpoint_id}-rgbd-relay-in",
            settings=settings,
            expected_source_id=expected_source_id,
            expected_boot_id=expected_boot_id,
        )
        self._publisher = DdsRgbdEdgePublisher(
            self.broker_topic,
            endpoint_id=endpoint_id,
            settings=settings,
            policy=policy,
            source_id=expected_source_id or endpoint_id,
            source_boot_id=expected_boot_id,
        )
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="elesim-rgbd-relay", daemon=True)

    def start(self) -> None:
        self._subscriber.connect()
        if not self._thread.is_alive():
            self._stop.clear()
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._subscriber.close()
        self._publisher.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                sample = self._subscriber.recv_latest(timeout_ms=100)
                if sample is not None:
                    self._publisher.submit(sample)
            except Exception as exc:
                self._stop.wait(0.1)
                if not self._stop.is_set():
                    # Keep the relay alive across one malformed/latest sample;
                    # the bounded subscriber will drop it on the next spin.
                    self._publisher.broker.last_error = f"{exc.__class__.__name__}: {exc}"[:512]


__all__ = [
    "DdsRgbdEdgePublisher", "DdsRgbdRelay", "EncodedRgbdFrame", "RgbdCodec", "RgbdEdgeBroker",
    "RgbdEdgeError", "RgbdEdgeStats", "RgbdEncoder", "RgbdEncodingPolicy",
    "encoded_frame_from_message", "encode_rgbd_frame",
]
