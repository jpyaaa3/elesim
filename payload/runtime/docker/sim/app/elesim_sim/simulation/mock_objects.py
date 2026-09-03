"""Safe, dependency-free loading of mock objects for Sim.

The mock object boundary deliberately accepts only a small, local subset of
Wavefront OBJ.  It produces an immutable geometry artifact; Genesis and DDS
are intentionally not involved here.  A later scan/reconstruction producer
can target the same artifact descriptor without changing the Sim spawn path.
"""

from __future__ import annotations

import hashlib
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping


class MockObjectError(ValueError):
    """The mock catalog entry is unsafe, malformed, or outside its bounds."""


@dataclass(frozen=True)
class MockObjectLimits:
    """Hard limits applied before an OBJ becomes a runtime artifact."""

    max_file_bytes: int = 8 * 1024 * 1024
    max_vertices: int = 100_000
    max_faces: int = 100_000
    max_line_bytes: int = 16 * 1024
    max_polygon_points: int = 64
    min_polygon_area: float = 1e-12

    def __post_init__(self) -> None:
        if any(
            value <= 0
            for value in (
                self.max_file_bytes,
                self.max_vertices,
                self.max_faces,
                self.max_line_bytes,
                self.max_polygon_points,
            )
        ):
            raise ValueError("mock object limits must be positive")
        if not math.isfinite(self.min_polygon_area) or self.min_polygon_area <= 0:
            raise ValueError("min_polygon_area must be finite and positive")


Point3 = tuple[float, float, float]
Point2 = tuple[float, float]
Face = tuple[int, ...]


@dataclass(frozen=True)
class MockObjectArtifact:
    """Immutable, validated geometry accepted by the Sim mock spawner."""

    asset_id: str
    revision: str
    sha256: str
    units: str
    vertices: tuple[Point3, ...]
    faces: tuple[Face, ...]
    polygon_xz: tuple[Point2, ...]

    @property
    def polygon(self) -> tuple[Point2, ...]:
        """Compatibility alias for the local XZ footprint."""

        return self.polygon_xz

    @property
    def vertex_count(self) -> int:
        return len(self.vertices)

    @property
    def face_count(self) -> int:
        return len(self.faces)

    def descriptor(self) -> dict[str, Any]:
        """Return a transport-safe identity/geometry descriptor.

        The descriptor carries no file path.  ``revision`` is content-derived
        and therefore becomes stale whenever the source OBJ changes.
        """

        return {
            "asset_id": self.asset_id,
            "revision": self.revision,
            "sha256": self.sha256,
            "units": self.units,
            "vertex_count": self.vertex_count,
            "face_count": self.face_count,
            "polygon_xz": [list(point) for point in self.polygon_xz],
        }

    def to_descriptor(self) -> dict[str, Any]:
        return self.descriptor()


def _error(path: Path, message: str) -> MockObjectError:
    return MockObjectError(f"{path.name}: {message}")


def _safe_filename(raw: str | Path) -> str:
    value = str(raw)
    path = Path(value)
    if not value or path.is_absolute() or "\\" in value:
        raise MockObjectError(f"unsafe mock object name: {value!r}")
    if path.name != value or any(part in {"", ".", ".."} for part in path.parts):
        raise MockObjectError(f"mock object must be a catalog basename: {value!r}")
    if path.suffix == "":
        value += ".obj"
    if Path(value).suffix != ".obj" or Path(value).name != value:
        raise MockObjectError(f"mock object must be a .obj basename: {value!r}")
    asset_id = Path(value).stem
    if (
        len(asset_id) > 64
        or asset_id in {".", ".."}
        or any(
            character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-"
            for character in asset_id
        )
    ):
        raise MockObjectError(f"mock object has an invalid asset id: {asset_id!r}")
    return value


def _parse_index(token: str, vertex_count: int, path: Path) -> int:
    raw = token.split("/", 1)[0]
    if not raw:
        raise _error(path, f"face has an empty vertex index: {token!r}")
    try:
        index = int(raw, 10)
    except ValueError as exc:
        raise _error(path, f"face index is not an integer: {token!r}") from exc
    if index == 0:
        raise _error(path, "face index is one-based and cannot be zero")
    resolved = index - 1 if index > 0 else vertex_count + index
    if resolved < 0 or resolved >= vertex_count:
        raise _error(path, f"face index out of range: {index}")
    return resolved


def _cross(a: Point2, b: Point2, c: Point2) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _convex_hull_xz(
    vertices: Iterable[Point3],
    *,
    limits: MockObjectLimits,
    path: Path,
) -> tuple[Point2, ...]:
    # Sorting a set gives a stable hull independent of OBJ vertex order.  A
    # 3-D cube naturally has duplicate XZ projections, so duplicates are
    # removed at this projection boundary rather than rejected as bad mesh
    # data.  The resulting polygon itself cannot contain duplicate points.
    points = sorted({(vertex[0], vertex[2]) for vertex in vertices})
    if len(points) < 3:
        raise _error(path, "local XZ projection is degenerate")

    lower: list[Point2] = []
    for point in points:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[Point2] = []
    for point in reversed(points):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    hull = tuple(lower[:-1] + upper[:-1])
    if len(hull) > limits.max_polygon_points:
        raise _error(path, f"local XZ polygon exceeds {limits.max_polygon_points} points")
    if len(hull) < 3 or len(set(hull)) != len(hull):
        raise _error(path, "local XZ polygon has duplicate or degenerate points")
    area2 = sum(
        point[0] * hull[(index + 1) % len(hull)][1]
        - point[1] * hull[(index + 1) % len(hull)][0]
        for index, point in enumerate(hull)
    )
    if not math.isfinite(area2) or abs(area2) * 0.5 < limits.min_polygon_area:
        raise _error(path, "local XZ polygon has zero or insufficient area")
    return hull


def project_artifact_xz(
    artifact: MockObjectArtifact,
    euler_deg: tuple[float, float, float],
    *,
    limits: MockObjectLimits | None = None,
) -> tuple[Point2, ...]:
    """Project an artifact into the world-oriented XZ planning plane.

    Genesis applies the same conventional XYZ Euler rotation to the visible
    entity.  Keeping this projection at the Sim artifact boundary prevents
    Pilot from planning against an unrotated silhouette while preserving the
    bounded, path-free DDS representation.
    """

    if len(euler_deg) != 3 or not all(math.isfinite(float(value)) for value in euler_deg):
        raise MockObjectError("mock object Euler rotation must contain three finite values")
    roll, pitch, yaw = (math.radians(float(value)) for value in euler_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # Rz(yaw) @ Ry(pitch) @ Rx(roll), matching scipy's XYZ Euler convention
    # used when the Genesis entity pose is applied in runtime.py.
    rotated: list[Point3] = []
    for x, y, z in artifact.vertices:
        x1, y1, z1 = x, cr * y - sr * z, sr * y + cr * z
        x2, y2, z2 = cp * x1 + sp * z1, y1, -sp * x1 + cp * z1
        values = (cy * x2 - sy * y2, sy * x2 + cy * y2, z2)
        rotated.append(tuple(0.0 if abs(value) < 1e-12 else value for value in values))  # type: ignore[arg-type]
    return _convex_hull_xz(
        rotated,
        limits=limits or MockObjectLimits(),
        path=Path(f"{artifact.asset_id}.obj"),
    )


def _load_path(path: Path, *, asset_id: str, limits: MockObjectLimits) -> MockObjectArtifact:
    if path.is_symlink() or not path.is_file():
        raise _error(path, "mock object must be a regular non-symlink file")
    try:
        size = path.stat().st_size
        if size > limits.max_file_bytes:
            raise _error(path, f"file exceeds {limits.max_file_bytes} bytes")
        payload = path.read_bytes()
    except OSError as exc:
        raise _error(path, f"cannot read file: {exc}") from exc
    if len(payload) != size:
        raise _error(path, "file changed while being read")
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise _error(path, "OBJ must be valid UTF-8 text") from exc

    vertices: list[Point3] = []
    raw_faces: list[tuple[tuple[str, ...], int]] = []
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        if len(raw_line.encode("utf-8")) > limits.max_line_bytes:
            raise _error(path, f"line {line_number} exceeds {limits.max_line_bytes} bytes")
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        kind = fields[0]
        if kind == "v":
            if len(fields) < 4:
                raise _error(path, f"line {line_number} has an incomplete vertex")
            if len(vertices) >= limits.max_vertices:
                raise _error(path, f"vertex count exceeds {limits.max_vertices}")
            try:
                vertex = tuple(float(value) for value in fields[1:4])
            except ValueError as exc:
                raise _error(path, f"line {line_number} has an invalid vertex") from exc
            if not all(math.isfinite(value) for value in vertex):
                raise _error(path, f"line {line_number} has a non-finite vertex")
            vertices.append(vertex)  # type: ignore[arg-type]
        elif kind == "f":
            if len(fields) < 4:
                raise _error(path, f"line {line_number} has a face with fewer than 3 vertices")
            if len(raw_faces) >= limits.max_faces:
                raise _error(path, f"face count exceeds {limits.max_faces}")
            raw_faces.append((tuple(fields[1:]), len(vertices)))

    if not vertices:
        raise _error(path, "OBJ contains no vertices")
    if not raw_faces:
        raise _error(path, "OBJ contains no faces")
    faces_list: list[Face] = []
    for raw_face, vertex_count in raw_faces:
        face = tuple(_parse_index(token, vertex_count, path) for token in raw_face)
        if len(set(face)) < 3:
            raise _error(path, "face must reference at least three distinct vertices")
        faces_list.append(face)
    faces = tuple(faces_list)
    polygon = _convex_hull_xz(vertices, limits=limits, path=path)
    digest = hashlib.sha256(payload).hexdigest()
    return MockObjectArtifact(
        asset_id=asset_id,
        revision=f"sha256:{digest}",
        sha256=digest,
        units="m",
        vertices=tuple(vertices),
        faces=faces,
        polygon_xz=polygon,
    )


class MockObjectCatalog:
    """Resolve direct-child OBJ assets without allowing path escape."""

    def __init__(self, root: str | Path, *, limits: MockObjectLimits | None = None) -> None:
        raw_root = Path(root)
        if raw_root.is_symlink():
            raise MockObjectError("mock catalog root may not be a symlink")
        if not raw_root.is_dir():
            raise MockObjectError(f"mock catalog root is not a directory: {raw_root}")
        self.root = raw_root.resolve()
        self.limits = limits or MockObjectLimits()
        # Runtime geometry is immutable for one Sim process.  The first load
        # freezes the exact artifact used to create the Genesis entity so a
        # later on-disk edit cannot make status hashes describe different
        # geometry from the already-built scene.
        self._artifacts: dict[str, MockObjectArtifact] = {}

    def assets(self) -> tuple[str, ...]:
        names: list[str] = []
        try:
            entries = sorted(self.root.iterdir(), key=lambda item: item.name)
        except OSError as exc:
            raise MockObjectError(f"cannot list mock catalog: {exc}") from exc
        for entry in entries:
            if entry.is_symlink():
                raise MockObjectError(f"mock catalog symlinks are not allowed: {entry.name}")
            if entry.is_file() and entry.suffix == ".obj":
                _safe_filename(entry.name)
                names.append(entry.name)
        return tuple(names)

    def load(self, name: str | Path) -> MockObjectArtifact:
        filename = _safe_filename(name)
        cached = self._artifacts.get(filename)
        if cached is not None:
            return cached
        path = self.root / filename
        if path.parent != self.root:
            raise MockObjectError(f"mock object escapes catalog: {name!r}")
        artifact = _load_path(path, asset_id=Path(filename).stem, limits=self.limits)
        self._artifacts[filename] = artifact
        return artifact

    def load_asset(self, name: str | Path) -> MockObjectArtifact:
        return self.load(name)


def load_mock_object(
    root: str | Path,
    name: str | Path,
    *,
    limits: MockObjectLimits | None = None,
) -> MockObjectArtifact:
    """Load one safe mock OBJ from ``root``."""

    return MockObjectCatalog(root, limits=limits).load(name)


def resolve_mock_object_catalog_root(source_root: str | Path) -> Path:
    """Resolve source/deployed catalog without importing the Genesis runtime."""

    configured = os.environ.get("ELESIM_SIM_MOCK_OBJECT_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser()
    candidates = (
        Path(source_root) / "payload/data/models/objects",
        Path(source_root) / "data/models/objects",
        Path("/opt/elesim/data/models/objects"),
    )
    return next((path for path in candidates if path.is_dir()), candidates[0])


__all__ = [
    "MockObjectArtifact",
    "MockObjectCatalog",
    "MockObjectError",
    "MockObjectLimits",
    "load_mock_object",
    "project_artifact_xz",
    "resolve_mock_object_catalog_root",
]
