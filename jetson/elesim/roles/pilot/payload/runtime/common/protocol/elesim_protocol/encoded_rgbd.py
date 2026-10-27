"""Bounded encoded RGB-D wire values.

The raw :mod:`elesim_protocol.rgbd` types remain available for local camera
work.  This module defines the value that is safe to put on an inter-process
or inter-host DDS link: a bounded payload with an explicit codec and all
metadata required to decode it.  Compression here deliberately uses only the
Python standard library so protocol validation can run without a camera SDK.
Production encoders may replace ``zlib`` with a hardware codec while keeping
the same wire metadata and limits.
"""

from __future__ import annotations

import math
import time
import uuid
import zlib
from dataclasses import dataclass
from threading import Lock
from typing import Any, Callable, Optional

from .dds_transport import DdsRuntimeSettings, DdsTransportError
from .rgbd import _RgbdRosEndpoint, _stamp, _timestamp

MAX_ENCODED_PAYLOAD_BYTES = 4 * 1024 * 1024
MAX_RGBD_PIXELS = 8192 * 8192
MAX_CODEC_NAME = 16
MAX_ENCODING_NAME = 16
MAX_CALIBRATION_ID = 128

SUPPORTED_CODECS = frozenset({"raw", "zlib", "jpeg", "png"})
SUPPORTED_COLOR_ENCODINGS = frozenset({"bgr8", "rgb8"})
SUPPORTED_DEPTH_ENCODINGS = frozenset({"16uc1", "32fc1"})


def _codec(value: object, label: str) -> str:
    result = str(value).strip().lower()
    if len(result) > MAX_CODEC_NAME:
        raise ValueError(f"{label} codec name is too long")
    return result


def encode_payload(
    payload: bytes | bytearray | memoryview,
    codec: str = "zlib",
    *,
    zlib_level: int = 1,
) -> bytes:
    """Encode one bounded payload using a protocol-supported codec."""

    selected = _codec(codec, "RGB-D")
    raw = bytes(payload)
    if len(raw) > MAX_ENCODED_PAYLOAD_BYTES:
        raise ValueError("RGB-D raw payload exceeds 4 MiB bound")
    if selected == "raw":
        encoded = raw
    elif selected == "zlib":
        encoded = zlib.compress(raw, level=max(0, min(9, int(zlib_level))))
    else:
        raise ValueError(f"unsupported RGB-D codec: {selected!r}")
    if len(encoded) > MAX_ENCODED_PAYLOAD_BYTES:
        raise ValueError("RGB-D encoded payload exceeds 4 MiB bound")
    return encoded


def decode_payload(payload: bytes | bytearray | memoryview, codec: str = "zlib") -> bytes:
    """Decode one bounded payload and reject decompression bombs."""

    selected = _codec(codec, "RGB-D")
    encoded = bytes(payload)
    if len(encoded) > MAX_ENCODED_PAYLOAD_BYTES:
        raise ValueError("RGB-D encoded payload exceeds 4 MiB bound")
    if selected == "raw":
        decoded = encoded
    elif selected == "zlib":
        try:
            decompressor = zlib.decompressobj()
            decoded = decompressor.decompress(encoded, MAX_ENCODED_PAYLOAD_BYTES + 1)
            if len(decoded) > MAX_ENCODED_PAYLOAD_BYTES or decompressor.unconsumed_tail:
                raise ValueError("RGB-D decoded payload exceeds 4 MiB bound")
            decoded += decompressor.flush(MAX_ENCODED_PAYLOAD_BYTES + 1 - len(decoded))
        except zlib.error as exc:
            raise ValueError("invalid zlib RGB-D payload") from exc
    elif selected in {"jpeg", "png"}:
        raise ValueError(f"{selected} is an image codec; use decode_image_payload")
    else:
        raise ValueError(f"unsupported RGB-D codec: {selected!r}")
    if len(decoded) > MAX_ENCODED_PAYLOAD_BYTES:
        raise ValueError("RGB-D decoded payload exceeds 4 MiB bound")
    return decoded


def _cv2() -> Any:
    """Load OpenCV lazily so protocol-only validation stays lightweight."""

    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - depends on deployment image
        raise ValueError(
            "JPEG/PNG RGB-D payloads require OpenCV in the active role image"
        ) from exc
    return cv2


def encode_image_payload(
    image: Any,
    codec: str,
    *,
    encoding: str,
    quality: int = 85,
    zlib_level: int = 1,
) -> bytes:
    """Encode one HxW RGB/depth array for the bounded wire value.

    JPEG is intentionally limited to color images.  Depth remains lossless
    and normally uses zlib; PNG is accepted for uint16 depth when a consumer
    prefers an image container.  OpenCV is imported only at the edge where a
    real image codec is requested.
    """

    selected = _codec(codec, "RGB-D")
    normalized_encoding = str(encoding).strip().lower()
    if selected in {"raw", "zlib"}:
        try:
            raw = image.tobytes(order="C")
        except AttributeError:
            raw = bytes(image)
        return (
            encode_payload(raw, selected, zlib_level=zlib_level)
            if selected == "zlib"
            else encode_payload(raw, "raw")
        )
    cv2 = _cv2()
    array = image
    if selected == "jpeg":
        if normalized_encoding not in SUPPORTED_COLOR_ENCODINGS:
            raise ValueError("JPEG RGB-D payloads require bgr8 or rgb8")
        if getattr(array, "dtype", None) is None or str(array.dtype) != "uint8":
            raise ValueError("JPEG RGB-D color input must be uint8")
        if normalized_encoding == "rgb8":
            array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(
            ".jpg", array, [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, int(quality)))]
        )
    elif selected == "png":
        if normalized_encoding not in SUPPORTED_DEPTH_ENCODINGS:
            raise ValueError("PNG RGB-D payloads require 16uc1 or 32fc1 depth")
        if normalized_encoding == "32fc1":
            raise ValueError("PNG RGB-D depth input must be uint16; use zlib for float depth")
        if getattr(array, "dtype", None) is None or str(array.dtype) != "uint16":
            raise ValueError("PNG RGB-D depth input must be uint16")
        ok, encoded = cv2.imencode(".png", array)
    else:  # pragma: no cover - guarded by SUPPORTED_CODECS
        raise ValueError(f"unsupported RGB-D codec: {selected!r}")
    if not ok:
        raise ValueError(f"OpenCV failed to encode RGB-D {selected} payload")
    if selected in {"jpeg", "png"}:
        # Image codecs are already compressed; do not wrap them in zlib.
        return encode_payload(encoded.tobytes(), "raw")


def decode_image_payload(
    payload: bytes | bytearray | memoryview,
    codec: str,
    *,
    width: int,
    height: int,
    encoding: str,
) -> Any:
    """Decode a wire payload to a contiguous NumPy image and validate shape."""

    selected = _codec(codec, "RGB-D")
    normalized_encoding = str(encoding).strip().lower()
    if selected in {"raw", "zlib"}:
        raw = decode_payload(payload, selected)
        try:
            import numpy as np
        except ImportError as exc:  # pragma: no cover
            raise ValueError("NumPy is required to decode RGB-D payloads") from exc
        dtype = np.uint8 if normalized_encoding in SUPPORTED_COLOR_ENCODINGS else (
            np.uint16 if normalized_encoding == "16uc1" else np.float32
        )
        channels = 3 if normalized_encoding in SUPPORTED_COLOR_ENCODINGS else 1
        expected = int(width) * int(height) * channels * np.dtype(dtype).itemsize
        if len(raw) != expected:
            raise ValueError("decoded RGB-D payload size does not match dimensions")
        shape = (int(height), int(width), channels) if channels == 3 else (int(height), int(width))
        return np.ascontiguousarray(np.frombuffer(raw, dtype=dtype).reshape(shape))
    cv2 = _cv2()
    import numpy as np

    encoded = np.frombuffer(bytes(payload), dtype=np.uint8)
    flag = cv2.IMREAD_COLOR if normalized_encoding in SUPPORTED_COLOR_ENCODINGS else cv2.IMREAD_UNCHANGED
    image = cv2.imdecode(encoded, flag)
    if image is None:
        raise ValueError(f"invalid {selected} RGB-D image payload")
    if int(image.shape[1]) != int(width) or int(image.shape[0]) != int(height):
        raise ValueError("decoded RGB-D image dimensions do not match metadata")
    if normalized_encoding in SUPPORTED_COLOR_ENCODINGS:
        if image.ndim != 3 or int(image.shape[2]) != 3:
            raise ValueError("decoded RGB-D color image is not three-channel")
        if normalized_encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif normalized_encoding == "16uc1" and (image.ndim != 2 or image.dtype != np.uint16):
        raise ValueError("decoded RGB-D depth image is not uint16 single-channel")
    return np.ascontiguousarray(image)


def encoded_frame_from_rgbd(
    frame: Any,
    *,
    source_id: str,
    source_boot_id: str,
    color_codec: str = "jpeg",
    depth_codec: str = "zlib",
    jpeg_quality: int = 85,
    zlib_level: int = 1,
    calibration_id: str = "",
) -> "EncodedRgbdFrame":
    """Encode a local :class:`RgbdFrame` without importing an application role."""

    try:
        import numpy as np

        color = np.ascontiguousarray(frame.color_bgr, dtype=np.uint8)
        raw_depth = getattr(frame, "depth_raw", None)
        depth = None if raw_depth is None else np.ascontiguousarray(raw_depth)
        if color.ndim != 3 or color.shape[2] != 3:
            raise ValueError("RGB-D color image must have shape HxWx3")
        if depth is not None and (depth.ndim != 2 or depth.shape != color.shape[:2]):
            raise ValueError("RGB-D depth image must match color dimensions")
        if depth is not None and depth.dtype not in (np.dtype(np.uint16), np.dtype(np.float32)):
            depth = np.ascontiguousarray(depth, dtype=np.uint16)
        intrinsics = frame.intrinsics
        has_depth = depth is not None
        depth_encoding = "16uc1" if has_depth and depth.dtype == np.uint16 else "32fc1" if has_depth else ""
        result = EncodedRgbdFrame(
            source_id=str(source_id),
            source_boot_id=str(source_boot_id),
            seq=int(getattr(frame, "seq", 0)),
            ts=float(getattr(frame, "ts", 0.0)),
            width=int(color.shape[1]),
            height=int(color.shape[0]),
            color_codec=str(color_codec).strip().lower(),
            color_encoding="bgr8",
            color_payload=encode_image_payload(
                color,
                color_codec,
                encoding="bgr8",
                quality=jpeg_quality,
                zlib_level=zlib_level,
            ),
            depth_codec=str(depth_codec).strip().lower() if has_depth else "",
            depth_encoding=depth_encoding,
            depth_payload=encode_image_payload(
                depth,
                depth_codec,
                encoding=depth_encoding,
                quality=jpeg_quality,
                zlib_level=zlib_level,
            ) if has_depth else b"",
            depth_scale=float(frame.depth_scale) if has_depth else 0.0,
            metadata=RgbdEncodedMetadata(
                fx=float(intrinsics.fx),
                fy=float(intrinsics.fy),
                cx=float(intrinsics.cx),
                cy=float(intrinsics.cy),
                calibration_id=str(calibration_id),
                arm_q=getattr(frame, "arm_q", None),
                camera_world_origin=getattr(frame, "camera_world_origin", None),
                camera_world_look=getattr(frame, "camera_world_look", None),
                camera_world_right=getattr(frame, "camera_world_right", None),
            ),
            has_depth=has_depth,
        )
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"invalid local RGB-D frame: {exc}") from exc
    # The encoder already owns the arrays and dimensions.  Do not decode the
    # newly-compressed bytes a second time on the producer hot path.
    return validate_encoded_frame(result, decode_payloads=False)


def rgbd_from_encoded_frame(frame: "EncodedRgbdFrame") -> Any:
    """Decode one encoded frame to the application-neutral raw RGB-D value."""

    # Structural validation is cheap; the actual image decode below is the
    # single payload validation pass for a consumer.
    validate_encoded_frame(frame, decode_payloads=False)
    from .rgbd import RgbdFrame, RgbdIntrinsics

    color = decode_image_payload(
        frame.color_payload,
        frame.color_codec,
        width=frame.width,
        height=frame.height,
        encoding=frame.color_encoding,
    )
    depth = decode_image_payload(
        frame.depth_payload,
        frame.depth_codec,
        width=frame.width,
        height=frame.height,
        encoding=frame.depth_encoding,
    ) if frame.has_depth else None
    return RgbdFrame(
        color_bgr=color,
        depth_raw=depth,
        depth_scale=float(frame.depth_scale),
        intrinsics=RgbdIntrinsics(
            fx=float(frame.metadata.fx),
            fy=float(frame.metadata.fy),
            cx=float(frame.metadata.cx),
            cy=float(frame.metadata.cy),
            width=int(frame.width),
            height=int(frame.height),
        ),
        seq=int(frame.seq),
        ts=float(frame.ts),
        arm_q=frame.metadata.arm_q,
        camera_world_origin=frame.metadata.camera_world_origin,
        camera_world_look=frame.metadata.camera_world_look,
        camera_world_right=frame.metadata.camera_world_right,
    )


@dataclass(frozen=True)
class RgbdEncodedMetadata:
    """Calibration and optional robot pose carried with an encoded frame."""

    fx: float
    fy: float
    cx: float
    cy: float
    calibration_id: str = ""
    arm_q: Optional[tuple[float, float, float, float]] = None
    camera_world_origin: Optional[tuple[float, float, float]] = None
    camera_world_look: Optional[tuple[float, float, float]] = None
    camera_world_right: Optional[tuple[float, float, float]] = None


@dataclass(frozen=True)
class EncodedRgbdFrame:
    """A bounded, self-describing RGB-D sample for the DDS wire contract."""

    source_id: str
    source_boot_id: str
    seq: int
    ts: float
    width: int
    height: int
    color_codec: str
    color_encoding: str
    color_payload: bytes
    depth_codec: str
    depth_encoding: str
    depth_payload: bytes
    depth_scale: float
    metadata: RgbdEncodedMetadata
    has_depth: Optional[bool] = None

    def __post_init__(self) -> None:
        if self.has_depth is None:
            absent = (
                not self.depth_payload
                and not str(self.depth_codec).strip()
                and not str(self.depth_encoding).strip()
            )
            object.__setattr__(self, "has_depth", not absent)
            if absent:
                # A hand-built frame may carry the color defaults' depth scale
                # while omitting depth entirely.  Normalize that shorthand to
                # the wire representation instead of making callers repeat
                # ``depth_scale=0``.
                object.__setattr__(self, "depth_scale", 0.0)

    def validate(self) -> "EncodedRgbdFrame":
        return validate_encoded_frame(self)


def _vector(value: object, size: int, label: str) -> None:
    if value is None:
        return
    try:
        values = tuple(float(component) for component in value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"RGB-D {label} must contain {size} finite values") from exc
    if len(values) != size or not all(math.isfinite(component) for component in values):
        raise ValueError(f"RGB-D {label} must contain {size} finite values")


def _vector_values(value: object, default: tuple[float, ...]) -> list[float]:
    """Return a metadata vector without evaluating it for truthiness.

    Camera pose values commonly arrive as NumPy arrays.  NumPy deliberately
    rejects boolean evaluation of arrays with more than one element, so
    ``value or default`` is not a valid optional-vector operation here.
    Validation has already checked the vector shape and finiteness before this
    helper is used by the publisher; this function only performs the wire
    conversion and handles the genuinely absent (``None``) case.
    """

    if value is None:
        return list(default)
    try:
        return [float(component) for component in value]  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("RGB-D metadata vector is not iterable") from exc


def validate_encoded_frame(
    frame: EncodedRgbdFrame,
    *,
    decode_payloads: bool = True,
) -> EncodedRgbdFrame:
    """Validate all fields that map to the bounded ROS message.

    ``decode_payloads=False`` is used at publishers and DDS callbacks. It
    still checks every scalar, codec, encoding, and byte bound, but defers
    expensive JPEG/zlib decoding to the consumer that needs pixels. This
    avoids decoding one frame twice on the publish/receive path.
    """

    if not isinstance(frame, EncodedRgbdFrame):
        raise TypeError("RGB-D frame must be EncodedRgbdFrame")
    if not str(frame.source_id).strip() or not str(frame.source_boot_id).strip():
        raise ValueError("RGB-D source ID and boot ID are required")
    if not 0 <= int(frame.seq) <= (1 << 64) - 1:
        raise ValueError("RGB-D sequence must fit uint64")
    if not math.isfinite(float(frame.ts)):
        raise ValueError("RGB-D timestamp must be finite")
    width, height = int(frame.width), int(frame.height)
    if width <= 0 or height <= 0 or width * height > MAX_RGBD_PIXELS:
        raise ValueError("RGB-D dimensions must be positive and bounded")
    color_codec = _codec(frame.color_codec, "color")
    if color_codec not in SUPPORTED_CODECS:
        raise ValueError(f"unsupported color codec: {color_codec!r}")
    color_encoding = str(frame.color_encoding).strip().lower()
    if color_encoding not in SUPPORTED_COLOR_ENCODINGS:
        raise ValueError(f"unsupported color encoding: {color_encoding!r}")
    color_payload = bytes(frame.color_payload)
    if not color_payload:
        raise ValueError("RGB-D color payload is required")
    if len(color_payload) > MAX_ENCODED_PAYLOAD_BYTES:
        raise ValueError("RGB-D color payload exceeds 4 MiB bound")
    if decode_payloads:
        try:
            if color_codec in {"jpeg", "png"}:
                decode_image_payload(
                    color_payload,
                    color_codec,
                    width=width,
                    height=height,
                    encoding=color_encoding,
                )
            else:
                decoded_color_size = len(decode_payload(color_payload, color_codec))
                expected_color_size = width * height * 3
                if decoded_color_size != expected_color_size:
                    raise ValueError("decoded color payload size does not match dimensions")
        except ValueError as exc:
            raise ValueError(f"invalid color payload: {exc}") from exc
    elif color_codec == "raw" and len(color_payload) != width * height * 3:
        raise ValueError("raw color payload size does not match dimensions")
    depth_codec = _codec(frame.depth_codec, "depth")
    depth_payload = bytes(frame.depth_payload)
    has_depth = bool(frame.has_depth)
    if has_depth:
        if depth_codec not in SUPPORTED_CODECS:
            raise ValueError(f"unsupported depth codec: {depth_codec!r}")
        if not depth_payload:
            raise ValueError("depth payload is required when has_depth is true")
        if str(frame.depth_encoding).strip().lower() not in SUPPORTED_DEPTH_ENCODINGS:
            raise ValueError(f"unsupported depth encoding: {frame.depth_encoding!r}")
        scale = float(frame.depth_scale)
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError("depth scale must be positive when depth is present")
    else:
        if depth_codec or depth_payload or str(frame.depth_encoding).strip():
            raise ValueError(
                "depth payload, codec and encoding must be empty when has_depth is false"
            )
        if float(frame.depth_scale) != 0.0:
            raise ValueError("depth scale must be zero when depth is absent")
    if len(depth_payload) > MAX_ENCODED_PAYLOAD_BYTES:
        raise ValueError("RGB-D depth payload exceeds 4 MiB bound")
    if has_depth:
        expected_depth_size = width * height * (
            2 if str(frame.depth_encoding).strip().lower() == "16uc1" else 4
        )
        if decode_payloads:
            try:
                if depth_codec in {"jpeg", "png"}:
                    decode_image_payload(
                        depth_payload,
                        depth_codec,
                        width=width,
                        height=height,
                        encoding=str(frame.depth_encoding).strip().lower(),
                    )
                else:
                    decoded_depth_size = len(decode_payload(depth_payload, depth_codec))
                    if decoded_depth_size != expected_depth_size:
                        raise ValueError("decoded depth payload size does not match dimensions")
            except ValueError as exc:
                raise ValueError(f"invalid depth payload: {exc}") from exc
        elif depth_codec == "raw" and len(depth_payload) != expected_depth_size:
            raise ValueError("raw depth payload size does not match dimensions")

    metadata = frame.metadata
    if not isinstance(metadata, RgbdEncodedMetadata):
        raise TypeError("RGB-D metadata must be RgbdEncodedMetadata")
    for name in ("fx", "fy", "cx", "cy"):
        value = float(getattr(metadata, name))
        if not math.isfinite(value) or name in {"fx", "fy"} and value <= 0.0:
            raise ValueError(f"RGB-D intrinsics {name} are invalid")
    calibration_id = str(metadata.calibration_id)
    if len(calibration_id) > MAX_CALIBRATION_ID:
        raise ValueError("RGB-D calibration ID is too long")
    _vector(metadata.arm_q, 4, "arm_q")
    for name in ("camera_world_origin", "camera_world_look", "camera_world_right"):
        _vector(getattr(metadata, name), 3, name)
    return frame


class DdsEncodedRgbdPublisher(_RgbdRosEndpoint):
    """Publish bounded encoded RGB-D samples on a dedicated DDS topic."""

    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str,
        settings: Optional[DdsRuntimeSettings] = None,
        boot_id: str = "",
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        super().__init__(
            topic=topic,
            endpoint_id=endpoint_id,
            settings=settings,
            node_suffix="encoded_rgbd_publisher",
        )
        try:
            from elesim_interfaces.msg import EncodedRgbdFrame as RosEncodedRgbdFrame
        except ImportError as exc:
            self.close()
            raise DdsTransportError(
                "ROS 2 encoded RGB-D interface is unavailable. Build the "
                "EleSim interfaces overlay before starting this role."
            ) from exc
        self._RosEncodedRgbdFrame = RosEncodedRgbdFrame
        self.boot_id = str(boot_id).strip() or uuid.uuid4().hex
        self.wall_clock = wall_clock
        self.published = 0
        self.dropped = 0
        self.last_drop_reason = ""
        self._publisher = self._node.create_publisher(
            self._RosEncodedRgbdFrame, self.topic, self._qos
        )

    @property
    def bound_endpoint(self) -> str:
        return self.topic

    def publish(self, frame: EncodedRgbdFrame) -> bool:
        if self._closed:
            return False
        try:
            validate_encoded_frame(frame, decode_payloads=False)
        except (TypeError, ValueError, OverflowError) as exc:
            self.dropped += 1
            self.last_drop_reason = f"{exc.__class__.__name__}: {exc}"[:512]
            return False
        # Message construction and RMW publication are intentionally outside
        # the malformed-input boundary, matching DdsRgbdPublisher semantics.
        message = self._RosEncodedRgbdFrame()
        message.source.endpoint_id = str(frame.source_id)
        message.source.boot_id = str(frame.source_boot_id)
        message.frame_sequence = int(frame.seq)
        _stamp(message.capture_time, float(frame.ts) or self.wall_clock())
        message.width = int(frame.width)
        message.height = int(frame.height)
        message.color_codec = str(frame.color_codec)
        message.color_encoding = str(frame.color_encoding)
        message.color_data = list(frame.color_payload)
        message.has_depth = bool(frame.has_depth)
        message.depth_codec = str(frame.depth_codec)
        message.depth_encoding = str(frame.depth_encoding)
        message.depth_data = list(frame.depth_payload)
        message.depth_scale = float(frame.depth_scale)
        metadata = frame.metadata
        message.fx = float(metadata.fx)
        message.fy = float(metadata.fy)
        message.cx = float(metadata.cx)
        message.cy = float(metadata.cy)
        message.calibration_id = str(metadata.calibration_id)
        message.has_arm_q = metadata.arm_q is not None
        message.arm_q = _vector_values(metadata.arm_q, (0.0, 0.0, 0.0, 0.0))
        pose = (
            metadata.camera_world_origin,
            metadata.camera_world_look,
            metadata.camera_world_right,
        )
        message.has_camera_pose = all(value is not None for value in pose)
        message.camera_world_origin = _vector_values(
            metadata.camera_world_origin, (0.0, 0.0, 0.0)
        )
        message.camera_world_look = _vector_values(
            metadata.camera_world_look, (0.0, 0.0, 0.0)
        )
        message.camera_world_right = _vector_values(
            metadata.camera_world_right, (0.0, 0.0, 0.0)
        )
        self._publisher.publish(message)
        self.published += 1
        self.last_drop_reason = ""
        return True


class DdsEncodedRgbdSubscriber(_RgbdRosEndpoint):
    """Receive the newest bounded encoded RGB-D sample."""

    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str,
        settings: Optional[DdsRuntimeSettings] = None,
        expected_source_id: str = "",
        expected_boot_id: str = "",
    ) -> None:
        super().__init__(
            topic=topic,
            endpoint_id=endpoint_id,
            settings=settings,
            node_suffix="encoded_rgbd_subscriber",
        )
        try:
            from elesim_interfaces.msg import EncodedRgbdFrame as RosEncodedRgbdFrame
        except ImportError as exc:
            self.close()
            raise DdsTransportError(
                "ROS 2 encoded RGB-D interface is unavailable. Build the "
                "EleSim interfaces overlay before starting this role."
            ) from exc
        self._RosEncodedRgbdFrame = RosEncodedRgbdFrame
        self._lock = Lock()
        self._latest: Any = None
        self._expected_source_id = str(expected_source_id).strip()
        self._expected_boot_id = str(expected_boot_id).strip()
        if self._expected_boot_id and not self._expected_source_id:
            raise ValueError("expected encoded RGB-D boot ID requires source ID")
        self._subscription = self._node.create_subscription(
            self._RosEncodedRgbdFrame, self.topic, self._receive, self._qos
        )

    def connect(self) -> None:
        if self._closed:
            raise DdsTransportError("encoded RGB-D subscriber is closed")

    def set_expected_source(self, endpoint_id: str, boot_id: str = "") -> None:
        source = str(endpoint_id).strip()
        boot = str(boot_id).strip()
        if boot and not source:
            raise ValueError("expected encoded RGB-D boot ID requires source ID")
        with self._lock:
            self._expected_source_id = source
            self._expected_boot_id = boot
            self._latest = None

    def recv_latest(self, *, timeout_ms: int = 500) -> Optional[EncodedRgbdFrame]:
        if self._closed:
            return None
        self.connect()
        with self._lock:
            self._latest = None
        self._executor.spin_once(timeout_sec=max(0, int(timeout_ms)) / 1000.0)
        for _ in range(16):
            with self._lock:
                before = self._latest
            self._executor.spin_once(timeout_sec=0.0)
            with self._lock:
                if self._latest is before:
                    break
        with self._lock:
            message = self._latest
            self._latest = None
        return None if message is None else self._decode(message)

    def _receive(self, message: Any) -> None:
        with self._lock:
            source_id = str(message.source.endpoint_id)
            boot_id = str(message.source.boot_id)
            if self._expected_source_id and source_id != self._expected_source_id:
                return
            if self._expected_boot_id and boot_id != self._expected_boot_id:
                return
            self._latest = message

    @staticmethod
    def _decode(message: Any) -> EncodedRgbdFrame:
        try:
            metadata = RgbdEncodedMetadata(
                fx=float(message.fx),
                fy=float(message.fy),
                cx=float(message.cx),
                cy=float(message.cy),
                calibration_id=str(message.calibration_id),
                arm_q=(
                    tuple(float(value) for value in message.arm_q)
                    if bool(message.has_arm_q)
                    else None
                ),
                camera_world_origin=(
                    tuple(float(value) for value in message.camera_world_origin)
                    if bool(message.has_camera_pose)
                    else None
                ),
                camera_world_look=(
                    tuple(float(value) for value in message.camera_world_look)
                    if bool(message.has_camera_pose)
                    else None
                ),
                camera_world_right=(
                    tuple(float(value) for value in message.camera_world_right)
                    if bool(message.has_camera_pose)
                    else None
                ),
            )
            frame = EncodedRgbdFrame(
                source_id=str(message.source.endpoint_id),
                source_boot_id=str(message.source.boot_id),
                seq=int(message.frame_sequence),
                ts=_timestamp(message.capture_time),
                width=int(message.width),
                height=int(message.height),
                color_codec=str(message.color_codec),
                color_encoding=str(message.color_encoding),
                color_payload=bytes(message.color_data),
                has_depth=bool(message.has_depth),
                depth_codec=str(message.depth_codec),
                depth_encoding=str(message.depth_encoding),
                depth_payload=bytes(message.depth_data),
                depth_scale=float(message.depth_scale),
                metadata=metadata,
            )
            return validate_encoded_frame(frame, decode_payloads=False)
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            raise DdsTransportError(f"invalid encoded RGB-D sample: {exc}") from exc


__all__ = [
    "MAX_ENCODED_PAYLOAD_BYTES",
    "MAX_RGBD_PIXELS",
    "SUPPORTED_CODECS",
    "EncodedRgbdFrame",
    "DdsEncodedRgbdPublisher",
    "DdsEncodedRgbdSubscriber",
    "RgbdEncodedMetadata",
    "decode_payload",
    "decode_image_payload",
    "encode_payload",
    "encode_image_payload",
    "encoded_frame_from_rgbd",
    "rgbd_from_encoded_frame",
    "validate_encoded_frame",
]
