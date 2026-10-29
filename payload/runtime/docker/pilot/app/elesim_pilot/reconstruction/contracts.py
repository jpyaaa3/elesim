"""Schema 1 model inputs/outputs. Arrays use metres, radians and scene axes."""
from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Protocol

import numpy as np

from elesim_protocol import SimMappingConfig, linear_effective_q_bounds
from elesim_protocol.scene_artifact import identifier
from elesim_pilot.vision.floor_calibration import camera_transform
from elesim_pilot.vision.partial_cloud import SensorObservation


MODEL_API_VERSION = 1


def array(value, *, shape=None, columns=None, maximum=100_000, dtype=float):
    result = np.array(value, dtype=dtype, copy=True)
    if shape is not None and result.shape != shape:
        raise ValueError(f"array shape {result.shape} does not match {shape}")
    if columns is not None and (result.ndim != 2 or result.shape[1] != columns or len(result) > maximum):
        raise ValueError("invalid or excessive array dimensions")
    if not np.all(np.isfinite(result)):
        raise ValueError("non-finite model data")
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class ImuSample:
    rpy_rad: tuple[float, float, float]
    source_boot_id: str
    calibration_id: str
    timestamp_s: float

    def __post_init__(self):
        rpy = array(self.rpy_rad, shape=(3,))
        if (any(not isinstance(value, str) or not 1 <= len(value) <= 128
                for value in (self.source_boot_id, self.calibration_id)) or not np.isfinite(self.timestamp_s)):
            raise ValueError("calibrated IMU needs identity, calibration and source time")
        object.__setattr__(self, "rpy_rad", tuple(float(value) for value in rpy))


@dataclass(frozen=True)
class Measurement:
    sensor: SensorObservation
    imu: ImuSample | None = None
    base_world_transform: np.ndarray | None = None

    def validated(self) -> "Measurement":
        s = self.sensor
        width, height = s.intrinsics.width, s.intrinsics.height
        if not (type(width) is int and type(height) is int and 0 < width*height <= 4_194_304):
            raise ValueError("invalid sensor dimensions")
        if s.color_bgr.dtype != np.uint8 or s.depth_raw.dtype not in (np.uint16, np.float32):
            raise ValueError("invalid RGB-D dtypes")
        color = array(s.color_bgr, shape=(height, width, 3), dtype=np.uint8)
        # Invalid depth pixels remain permitted; deprojection excludes them.
        depth = np.array(s.depth_raw, copy=True)
        if depth.shape != (height, width):
            raise ValueError("depth dimensions do not match intrinsics")
        depth.setflags(write=False)
        if (type(s.seq) is not int or s.seq < 0 or not np.isfinite(s.ts) or s.ts <= 0
                or not np.isfinite(s.depth_scale) or s.depth_scale <= 0
                or any(not isinstance(value, str) or not 1 <= len(value) <= 128
                       for value in (s.source_id, s.source_boot_id, s.calibration_id))):
            raise ValueError("invalid sensor identity, sequence, timestamp or scale")
        array((s.intrinsics.fx, s.intrinsics.fy, s.intrinsics.cx, s.intrinsics.cy), shape=(4,))
        if min(s.intrinsics.fx, s.intrinsics.fy) <= 0:
            raise ValueError("invalid camera focal lengths")
        array(s.encoder_q, shape=(4,))
        if self.imu is not None and (self.imu.source_boot_id != s.source_boot_id
                or not 0 <= s.ts - self.imu.timestamp_s <= .2):
            raise ValueError("IMU source boot or freshness does not match observation")
        base = None if self.base_world_transform is None else array(
            camera_transform(self.base_world_transform), shape=(4, 4))
        return Measurement(SensorObservation(color, depth, float(s.depth_scale), s.intrinsics, s.seq,
            float(s.ts), s.source_id, s.source_boot_id, s.calibration_id,
            tuple(float(value) for value in s.encoder_q)), self.imu, base)


@dataclass(frozen=True)
class Task:
    scene_id: str
    target_id: str
    frame_id: str = "world"
    operation: str = "inspect"
    roles: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self):
        for value in (self.scene_id, self.target_id, self.frame_id):
            identifier(value)
        if self.operation not in {"inspect", "pick", "wrap"}:
            raise ValueError("unsupported reconstruction task")
        roles = dict(self.roles)
        for key, role in roles.items():
            identifier(key)
            if role not in {"target", "obstacle", "contact"}:
                raise ValueError("invalid task object role")
            if (key == self.target_id) != (role == "target"):
                raise ValueError("only the selected target may have the target role")
        object.__setattr__(self, "roles", MappingProxyType(roles))

    def role(self, object_id):
        return self.roles.get(object_id, "target" if object_id == self.target_id else "obstacle")


@dataclass(frozen=True)
class InstanceMask:
    object_id: str
    label: str
    mask: np.ndarray
    confidence: float = 1.0
    kind: str = "instance"

    def __post_init__(self):
        identifier(self.object_id)
        if not isinstance(self.label, str) or not 1 <= len(self.label) <= 128:
            raise ValueError("invalid object label")
        if self.kind != "instance" or not np.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError("reconstruction requires an instance mask, never bbox fallback")
        value = np.asarray(self.mask)
        if value.dtype != bool or value.ndim != 2 or not value.any():
            raise ValueError("instance mask must be a nonempty boolean image")
        object.__setattr__(self, "mask", array(value, shape=value.shape, dtype=bool))


@dataclass(frozen=True)
class Pose:
    frame_id: str
    world_from_camera: np.ndarray

    def __post_init__(self):
        identifier(self.frame_id)
        object.__setattr__(self, "world_from_camera", array(camera_transform(self.world_from_camera), shape=(4, 4)))


@dataclass(frozen=True)
class ObjectCloud:
    object_id: str
    label: str
    role: str
    frame_id: str
    observed: np.ndarray

    def __post_init__(self):
        identifier(self.object_id)
        identifier(self.frame_id)
        if self.role not in {"target", "obstacle", "contact"}:
            raise ValueError("invalid cloud role")
        points = array(self.observed, columns=3)
        if not len(points) or np.any(np.abs(points) > 100):
            raise ValueError("invalid observed cloud in metres")
        object.__setattr__(self, "observed", points)


@dataclass(frozen=True)
class CompletedObject:
    cloud: ObjectCloud
    predicted: np.ndarray

    def __post_init__(self):
        points = array(self.predicted, columns=3)
        if np.any(np.abs(points) > 100):
            raise ValueError("predicted coordinates exceed scene bounds")
        object.__setattr__(self, "predicted", points)


@dataclass(frozen=True)
class Mesh:
    object_id: str
    frame_id: str
    vertices: np.ndarray
    triangles: np.ndarray

    def __post_init__(self):
        identifier(self.object_id)
        identifier(self.frame_id)
        vertices = array(self.vertices, columns=3)
        raw = np.asarray(self.triangles)
        if raw.dtype.kind not in "iu":
            raise ValueError("mesh faces must use integer indices")
        triangles = array(raw, columns=3, dtype=np.int64)
        if (not len(vertices) or not len(triangles) or np.any(np.abs(vertices) > 100)
                or triangles.min() < 0 or triangles.max() >= len(vertices)):
            raise ValueError("invalid mesh geometry")
        a, b, c = (vertices[triangles[:, i]] for i in range(3))
        if np.any(np.linalg.norm(np.cross(b-a, c-a), axis=1) <= 1e-12):
            raise ValueError("degenerate mesh triangle")
        object.__setattr__(self, "vertices", vertices)
        object.__setattr__(self, "triangles", triangles)


@dataclass(frozen=True)
class SceneObject:
    shape: CompletedObject
    mesh: Mesh

    def __post_init__(self):
        if not isinstance(self.shape, CompletedObject) or not isinstance(self.mesh, Mesh):
            raise ValueError("mesher must return a Mesh for the completed object")
        if (self.shape.cloud.object_id, self.shape.cloud.frame_id) != (self.mesh.object_id, self.mesh.frame_id):
            raise ValueError("mesher changed object identity or coordinate frame")


@dataclass(frozen=True)
class PlanningContext:
    task: Task
    objects: tuple[SceneObject, ...]
    start_q: np.ndarray
    mapping: SimMappingConfig
    source: tuple[str, str, str]
    seq: int
    timestamp_s: float

    def __post_init__(self):
        object.__setattr__(self, "start_q", array(self.start_q, shape=(4,)))


@dataclass(frozen=True)
class Plan:
    """A proposal for review; producing this value never executes motion."""
    target_id: str
    frame_id: str
    times_s: tuple[float, ...]
    q: np.ndarray
    contact_ids: tuple[str, ...] = ()

    def validate(self, context: PlanningContext):
        task, objects, mapping = context.task, context.objects, context.mapping
        values = array(self.q, columns=4, maximum=4000)
        times = array(self.times_s, shape=(len(values),))
        if len(values) < 2 or times[0] < 0 or times[-1] > 120 or np.any(np.diff(times) <= 0):
            raise ValueError("invalid timed plan")
        lower_linear, upper_linear = linear_effective_q_bounds(mapping)
        lower = [lower_linear, mapping.roll_q_min_rad, mapping.seg1_q_min_rad, mapping.seg2_q_min_rad]
        upper = [upper_linear, mapping.roll_q_max_rad, mapping.seg1_q_max_rad, mapping.seg2_q_max_rad]
        lower, upper = array(lower, shape=(4,)), array(upper, shape=(4,))
        if np.any(lower > upper):
            raise ValueError("invalid planning joint bounds")
        if np.any(values < lower) or np.any(values > upper):
            raise ValueError("plan exceeds configured joint bounds")
        if not np.allclose(values[0], context.start_q, rtol=0., atol=1e-6):
            raise ValueError("plan does not start at the measured joint state")
        if (self.target_id, self.frame_id) != (task.target_id, task.frame_id):
            raise ValueError("planner changed target or frame")
        available = {o.shape.cloud.object_id: o.shape.cloud.role for o in objects}
        if any(available.get(key) not in {"target", "contact"} for key in self.contact_ids):
            raise ValueError("planner requested contact with an obstacle or unknown object")
        return Plan(self.target_id, self.frame_id, tuple(times), values, tuple(self.contact_ids))


class Segmenter(Protocol):
    def segment(self, measurement: Measurement, task: Task) -> tuple[InstanceMask, ...]: ...


class PoseModel(Protocol):
    def estimate(self, measurement: Measurement, task: Task) -> Pose: ...


class PoseCorrection(Protocol):
    def correct(self, measurement: Measurement, pose: Pose, task: Task) -> Pose: ...


class CompletionModel(Protocol):
    def complete(self, cloud: ObjectCloud, task: Task) -> np.ndarray: ...


class Mesher(Protocol):
    def mesh(self, shape: CompletedObject, task: Task) -> Mesh: ...


class Planner(Protocol):
    def plan(self, context: PlanningContext) -> Plan: ...
