"""Portable scene artifact schema 1; file validation only, no DDS or simulator.

Vertices use metres in the declared scene frame. An artifact does not grant
motion authority or certify the inferred shape as a measured obstacle map.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re


SCENE_ARTIFACT_SCHEMA = 1
MAX_OBJECTS = 16
MAX_MESH_BYTES = 8 * 1024 * 1024
MAX_VERTICES = 100_000
MAX_FACES = 100_000
_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z")


def _invalid_constant(value):
    raise ValueError(f"non-finite JSON constant: {value}")


def identifier(value: object) -> str:
    if not isinstance(value, str) or not _ID.fullmatch(value):
        raise ValueError(f"invalid scene identifier: {value!r}")
    return value


def regular_file(path: Path, maximum: int) -> bytes:
    if any(p.is_symlink() for p in (path, *path.parents)) or not path.is_file():
        raise ValueError(f"artifact must be a regular file without symlink ancestors: {path}")
    if path.stat().st_size > maximum:
        raise ValueError(f"artifact exceeds byte budget: {path.name}")
    with path.open("rb") as handle:
        data = handle.read(maximum + 1)
    if len(data) > maximum:
        raise ValueError(f"artifact exceeds byte budget: {path.name}")
    return data


def validate_obj(data: bytes) -> tuple[int, int]:
    """The exporter contract is deliberately restricted to triangular OBJ."""
    if len(data) > MAX_MESH_BYTES:
        raise ValueError("mesh exceeds byte budget")
    vertices, faces = [], []
    for line in data.decode("utf-8").splitlines():
        if len(line) > 16384:
            raise ValueError("mesh line exceeds budget")
        fields = line.split()
        if not fields or fields[0].startswith("#"):
            continue
        if fields[0] == "v" and len(fields) == 4:
            vertex = tuple(float(v) for v in fields[1:])
            if not all(math.isfinite(v) and abs(v) <= 100 for v in vertex):
                raise ValueError("mesh coordinates must be finite metres within scene bounds")
            vertices.append(vertex)
        elif fields[0] == "f" and len(fields) == 4:
            face = tuple(int(v) - 1 for v in fields[1:])
            if len(set(face)) != 3 or min(face) < 0:
                raise ValueError("invalid triangular face")
            faces.append(face)
        else:
            raise ValueError("scene mesh accepts only vertices and triangular faces")
        if len(vertices) > MAX_VERTICES or len(faces) > MAX_FACES:
            raise ValueError("mesh geometry exceeds budget")
    if not vertices or not faces:
        raise ValueError("mesh must have vertices and triangular faces")
    for face in faces:
        if max(face) >= len(vertices):
            raise ValueError("face index outside mesh vertices")
        a, b, c = (vertices[i] for i in face)
        u, v = tuple(b[i] - a[i] for i in range(3)), tuple(c[i] - a[i] for i in range(3))
        cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        if sum(x*x for x in cross) <= 1e-24:
            raise ValueError("degenerate triangular face")
    return len(vertices), len(faces)


@dataclass(frozen=True)
class SceneMesh:
    object_id: str
    label: str
    role: str
    mesh_file: Path
    sha256: str
    observed_points: int
    predicted_points: int


@dataclass(frozen=True)
class SceneArtifact:
    scene_id: str
    frame_id: str
    revision: str
    objects: tuple[SceneMesh, ...]

    @classmethod
    def load(cls, manifest: str | Path, *, expected_frame: str = "world") -> "SceneArtifact":
        path = Path(manifest).expanduser().absolute()
        data = regular_file(path, 256 * 1024)
        raw = json.loads(data, parse_constant=_invalid_constant)
        json.dumps(raw, allow_nan=False)
        if not isinstance(raw, dict) or set(raw) != {
            "schema_version", "scene_id", "frame_id", "units", "objects", "provenance",
        }:
            raise ValueError("invalid scene manifest fields")
        if type(raw["schema_version"]) is not int or raw["schema_version"] != SCENE_ARTIFACT_SCHEMA:
            raise ValueError("unsupported scene artifact schema")
        identifier(raw["scene_id"])
        identifier(raw["frame_id"])
        if raw["units"] != "m" or raw["frame_id"] != expected_frame:
            raise ValueError("scene frame/units do not match the consumer")
        if not isinstance(raw["provenance"], dict):
            raise ValueError("scene provenance must be an object")
        if not isinstance(raw["objects"], list) or not 1 <= len(raw["objects"]) <= MAX_OBJECTS:
            raise ValueError("scene needs 1..16 objects")
        objects, seen = [], set()
        for obj in raw["objects"]:
            if not isinstance(obj, dict) or set(obj) != {
                "object_id", "label", "role", "mesh_file", "sha256", "observed_points", "predicted_points",
            }:
                raise ValueError("invalid scene object fields")
            object_id = identifier(obj["object_id"])
            if object_id in seen:
                raise ValueError("duplicate scene object identity")
            seen.add(object_id)
            if obj["mesh_file"] != object_id + ".obj":
                raise ValueError("mesh file must match the object identity")
            if not isinstance(obj["label"], str) or not 1 <= len(obj["label"]) <= 128:
                raise ValueError("invalid scene object label")
            if obj["role"] not in {"target", "obstacle", "contact"}:
                raise ValueError("invalid scene object role")
            if any(type(obj[key]) is not int or not 0 <= obj[key] <= 100_000
                   for key in ("observed_points", "predicted_points")) or obj["observed_points"] == 0:
                raise ValueError("invalid observed/predicted point counts")
            mesh_file = path.parent / obj["mesh_file"]
            mesh = regular_file(mesh_file, MAX_MESH_BYTES)
            if hashlib.sha256(mesh).hexdigest() != obj["sha256"]:
                raise ValueError("scene mesh digest mismatch")
            validate_obj(mesh)
            objects.append(SceneMesh(object_id, obj["label"], obj["role"], mesh_file,
                obj["sha256"], obj["observed_points"], obj["predicted_points"]))
        return cls(raw["scene_id"], raw["frame_id"], hashlib.sha256(data).hexdigest(), tuple(objects))
