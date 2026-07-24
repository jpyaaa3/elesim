"""Typed ROS 2/DDS RGB-D transport.

RGB-D is intentionally a latest-sample sensor stream: best effort, volatile,
and keep-last depth one.  It shares the process DDS security profile but owns a
dedicated rclpy context so applications that embed another ROS client (for
example Unitree's bridge) do not share global initialization state.
"""

from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass
from threading import Lock
from typing import Any, Callable, Optional

from .dds_transport import (
    DdsRuntimeSettings,
    DdsTransportError,
    peer_node_key,
    ros_name_component,
)


@dataclass(frozen=True)
class RgbdIntrinsicsSample:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class RgbdSample:
    color_bgr: Any
    depth_raw: Any
    depth_scale: float
    intrinsics: RgbdIntrinsicsSample
    seq: int
    ts: float
    source_id: str
    source_boot_id: str
    arm_q: Optional[tuple[float, float, float, float]] = None
    camera_world_origin: Optional[tuple[float, float, float]] = None
    camera_world_look: Optional[tuple[float, float, float]] = None
    camera_world_right: Optional[tuple[float, float, float]] = None


def _validate_topic(value: object) -> str:
    topic = str(value).strip()
    if (
        not topic.startswith("/")
        or len(topic) > 512
        or any(character.isspace() for character in topic)
    ):
        raise ValueError("DDS RGB-D topic must be an absolute ROS topic")
    return topic


def _stamp(message: Any, value: float) -> None:
    timestamp = float(value)
    if not math.isfinite(timestamp) or timestamp <= 0.0:
        timestamp = time.time()
    whole = int(timestamp)
    message.sec = whole
    message.nanosec = int((timestamp - whole) * 1_000_000_000)


def _timestamp(message: Any) -> float:
    return float(message.sec) + float(message.nanosec) / 1_000_000_000.0


class _RgbdRosEndpoint:
    def __init__(
        self,
        *,
        topic: str,
        endpoint_id: str,
        settings: Optional[DdsRuntimeSettings],
        node_suffix: str,
    ) -> None:
        self.topic = _validate_topic(topic)
        self.endpoint_id = str(endpoint_id).strip() or "rgbd"
        self.settings = settings or DdsRuntimeSettings.from_env(
            endpoint_id=self.endpoint_id
        )
        ros_args = self.settings.apply_environment()
        try:
            import rclpy
            from elesim_interfaces.msg import RgbdFrame as RosRgbdFrame
            from rclpy.context import Context
            from rclpy.duration import Duration
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
        except ImportError as exc:
            raise DdsTransportError(
                "ROS 2 RGB-D transport is unavailable. Source the ROS 2 and "
                "Elesim interfaces overlays before starting this role."
            ) from exc
        self._RosRgbdFrame = RosRgbdFrame
        self._context = Context()
        try:
            self._context.init(
                args=list(ros_args),
                domain_id=self.settings.domain_id,
            )
        except TypeError:
            self._context.init(args=list(ros_args))
        node_name = (
            f"elesim_rgbd_{node_suffix}_"
            f"{peer_node_key(self.endpoint_id)}_"
            f"{ros_name_component(uuid.uuid4().hex, fallback='node')}"
        )[:255]
        self._node = rclpy.create_node(
            node_name,
            context=self._context,
            namespace=self.settings.namespace,
            use_global_arguments=True,
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds=1.0),
        )
        self._closed = False

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._executor.remove_node(self._node)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        finally:
            try:
                self._executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
            if self._context.ok():
                self._context.shutdown()


class DdsRgbdPublisher(_RgbdRosEndpoint):
    """Publish application RGB-D frame objects as ``elesim_interfaces/RgbdFrame``."""

    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str,
        settings: Optional[DdsRuntimeSettings] = None,
        boot_id: str = "",
        send_depth: bool = True,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        super().__init__(
            topic=topic,
            endpoint_id=endpoint_id,
            settings=settings,
            node_suffix="publisher",
        )
        self.boot_id = str(boot_id).strip() or uuid.uuid4().hex
        self.send_depth = bool(send_depth)
        self.wall_clock = wall_clock
        self.published = 0
        self.dropped = 0
        self._publisher = self._node.create_publisher(
            self._RosRgbdFrame,
            self.topic,
            self._qos,
        )

    @property
    def bound_endpoint(self) -> str:
        return self.topic

    def publish(self, frame: Any) -> bool:
        if self._closed:
            return False
        try:
            import numpy as np

            color = np.ascontiguousarray(frame.color_bgr, dtype=np.uint8)
            if color.ndim != 3 or color.shape[2] != 3:
                raise ValueError("RGB-D color image must have shape HxWx3")
            height, width = int(color.shape[0]), int(color.shape[1])
            depth = np.ascontiguousarray(frame.depth_raw)
            if depth.ndim != 2 or depth.shape != (height, width):
                raise ValueError("RGB-D depth image must match color dimensions")
            if depth.dtype == np.uint16:
                depth_encoding = "16UC1"
            elif depth.dtype == np.float32:
                depth_encoding = "32FC1"
            else:
                depth = np.ascontiguousarray(depth, dtype=np.uint16)
                depth_encoding = "16UC1"

            message = self._RosRgbdFrame()
            message.source.endpoint_id = self.endpoint_id
            message.source.boot_id = self.boot_id
            message.frame_sequence = int(getattr(frame, "seq", 0))
            timestamp = float(getattr(frame, "ts", 0.0) or self.wall_clock())
            message.header.frame_id = self.endpoint_id
            _stamp(message.header.stamp, timestamp)

            message.color.header = message.header
            message.color.height = height
            message.color.width = width
            message.color.encoding = "bgr8"
            message.color.is_bigendian = False
            message.color.step = width * 3
            message.color.data = color.tobytes()

            message.depth.header = message.header
            message.depth.height = height
            message.depth.width = width
            message.depth.encoding = depth_encoding
            message.depth.is_bigendian = False
            message.depth.step = width * int(depth.dtype.itemsize)
            message.depth.data = depth.tobytes() if self.send_depth else b""

            intrinsics = frame.intrinsics
            message.camera_info.header = message.header
            message.camera_info.height = height
            message.camera_info.width = width
            message.camera_info.distortion_model = "plumb_bob"
            message.camera_info.d = []
            message.camera_info.k = [
                float(intrinsics.fx),
                0.0,
                float(intrinsics.cx),
                0.0,
                float(intrinsics.fy),
                float(intrinsics.cy),
                0.0,
                0.0,
                1.0,
            ]
            message.camera_info.r = [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ]
            message.camera_info.p = [
                float(intrinsics.fx),
                0.0,
                float(intrinsics.cx),
                0.0,
                0.0,
                float(intrinsics.fy),
                float(intrinsics.cy),
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
            message.depth_scale = float(frame.depth_scale)

            arm_q = getattr(frame, "arm_q", None)
            message.has_arm_q = arm_q is not None
            message.arm_q = (
                [float(value) for value in arm_q]
                if arm_q is not None
                else [0.0, 0.0, 0.0, 0.0]
            )
            origin = getattr(frame, "camera_world_origin", None)
            look = getattr(frame, "camera_world_look", None)
            right = getattr(frame, "camera_world_right", None)
            message.has_camera_pose = (
                origin is not None and look is not None and right is not None
            )
            message.camera_world_origin = (
                [float(value) for value in origin]
                if origin is not None
                else [0.0, 0.0, 0.0]
            )
            message.camera_world_look = (
                [float(value) for value in look]
                if look is not None
                else [0.0, 0.0, 0.0]
            )
            message.camera_world_right = (
                [float(value) for value in right]
                if right is not None
                else [0.0, 0.0, 0.0]
            )
            self._publisher.publish(message)
            self.published += 1
            return True
        except Exception:
            self.dropped += 1
            return False


class DdsRgbdSubscriber(_RgbdRosEndpoint):
    """Receive and decode the newest typed RGB-D DDS sample."""

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
            node_suffix="subscriber",
        )
        self._lock = Lock()
        self._latest: Any = None
        self._connected = True
        self._expected_source_id = str(expected_source_id).strip()
        self._expected_boot_id = str(expected_boot_id).strip()
        if self._expected_boot_id and not self._expected_source_id:
            raise ValueError(
                "expected DDS RGB-D boot ID requires an expected source ID"
            )
        self._subscription = self._node.create_subscription(
            self._RosRgbdFrame,
            self.topic,
            self._receive,
            self._qos,
        )

    def connect(self) -> None:
        if self._closed:
            raise DdsTransportError("DDS RGB-D subscriber is closed")
        self._connected = True

    def set_expected_source(
        self,
        endpoint_id: str,
        boot_id: str = "",
    ) -> None:
        """Atomically change the selected peer incarnation filter."""

        source = str(endpoint_id).strip()
        boot = str(boot_id).strip()
        if boot and not source:
            raise ValueError(
                "expected DDS RGB-D boot ID requires an expected source ID"
            )
        with self._lock:
            self._expected_source_id = source
            self._expected_boot_id = boot
            self._latest = None

    def close(self) -> None:
        self._connected = False
        super().close()

    def recv_latest(self, *, timeout_ms: int = 500) -> Optional[RgbdSample]:
        if self._closed:
            return None
        self.connect()
        with self._lock:
            self._latest = None
        self._executor.spin_once(
            timeout_sec=max(0, int(timeout_ms)) / 1000.0
        )
        # Drain callbacks already queued by DDS and retain only the newest.
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
            if (
                self._expected_source_id
                and source_id != self._expected_source_id
            ):
                return
            if self._expected_boot_id and boot_id != self._expected_boot_id:
                return
            self._latest = message

    @staticmethod
    def _decode(message: Any) -> RgbdSample:
        import numpy as np

        color_message = message.color
        height = int(color_message.height)
        width = int(color_message.width)
        if height <= 0 or width <= 0:
            raise DdsTransportError("DDS RGB-D color dimensions are invalid")
        encoding = str(color_message.encoding).lower()
        if encoding not in {"bgr8", "rgb8"}:
            raise DdsTransportError(
                f"unsupported DDS RGB-D color encoding: {encoding!r}"
            )
        row_step = int(color_message.step) or width * 3
        raw_color = np.frombuffer(bytes(color_message.data), dtype=np.uint8)
        if raw_color.size < height * row_step:
            raise DdsTransportError("DDS RGB-D color buffer is truncated")
        color = raw_color[: height * row_step].reshape(height, row_step)
        color = color[:, : width * 3].reshape(height, width, 3).copy()
        if encoding == "rgb8":
            color = np.ascontiguousarray(color[:, :, ::-1])

        depth_message = message.depth
        depth_encoding = str(depth_message.encoding).lower()
        if not depth_message.data:
            depth = np.zeros((height, width), dtype=np.uint16)
        else:
            if depth_encoding in {"16uc1", "mono16"}:
                dtype = np.dtype(">u2" if depth_message.is_bigendian else "<u2")
            elif depth_encoding == "32fc1":
                dtype = np.dtype(">f4" if depth_message.is_bigendian else "<f4")
            else:
                raise DdsTransportError(
                    f"unsupported DDS RGB-D depth encoding: {depth_encoding!r}"
                )
            depth_height = int(depth_message.height)
            depth_width = int(depth_message.width)
            if (depth_height, depth_width) != (height, width):
                raise DdsTransportError(
                    "DDS RGB-D color and depth dimensions differ"
                )
            row_step = int(depth_message.step) or depth_width * dtype.itemsize
            raw_depth = np.frombuffer(bytes(depth_message.data), dtype=np.uint8)
            if raw_depth.size < depth_height * row_step:
                raise DdsTransportError("DDS RGB-D depth buffer is truncated")
            rows = raw_depth[: depth_height * row_step].reshape(
                depth_height,
                row_step,
            )
            packed = np.ascontiguousarray(
                rows[:, : depth_width * dtype.itemsize]
            )
            depth = packed.view(dtype).reshape(depth_height, depth_width).copy()
            depth = depth.astype(dtype.newbyteorder("="), copy=False)

        camera_info = message.camera_info
        matrix = tuple(float(value) for value in camera_info.k)
        intrinsics = RgbdIntrinsicsSample(
            fx=matrix[0],
            fy=matrix[4],
            cx=matrix[2],
            cy=matrix[5],
            width=width,
            height=height,
        )
        arm_q = (
            tuple(float(value) for value in message.arm_q)
            if bool(message.has_arm_q)
            else None
        )
        pose = bool(message.has_camera_pose)
        return RgbdSample(
            color_bgr=color,
            depth_raw=depth,
            depth_scale=float(message.depth_scale),
            intrinsics=intrinsics,
            seq=int(message.frame_sequence),
            ts=_timestamp(message.header.stamp),
            source_id=str(message.source.endpoint_id),
            source_boot_id=str(message.source.boot_id),
            arm_q=arm_q,
            camera_world_origin=(
                tuple(float(value) for value in message.camera_world_origin)
                if pose
                else None
            ),
            camera_world_look=(
                tuple(float(value) for value in message.camera_world_look)
                if pose
                else None
            ),
            camera_world_right=(
                tuple(float(value) for value in message.camera_world_right)
                if pose
                else None
            ),
        )


__all__ = [
    "DdsRgbdPublisher",
    "DdsRgbdSubscriber",
    "RgbdIntrinsicsSample",
    "RgbdSample",
]
