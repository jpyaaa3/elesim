"""Runtime validation for a prebuilt sim model bundle."""

from __future__ import annotations

import hashlib
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping

from elesim_sim.vision.camera_profile import CAMERA_PROFILES, camera_profile


SCHEMA_VERSION = 1
BUNDLE_TYPE = "elesim.sim-model"
METADATA_NAME = "bundle.json"
ENTRYPOINTS = {
    "blueprint": "blueprint.json",
    "arm_urdf": "arm.urdf",
    "robot_urdf": "robot.urdf",
}
MODEL_BUNDLE_ENV = "ELESIM_MODEL_BUNDLE"


class ModelBundleError(ValueError):
    """The sim model bundle is unsafe, incomplete or corrupted."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _relative_path(raw: object, *, context: str) -> Path:
    value = str(raw or "").strip().replace("\\", "/")
    path = Path(value)
    if not value or path.is_absolute() or ".." in path.parts or "://" in value:
        raise ModelBundleError(f"unsafe {context} path: {value!r}")
    return path


def _payload_files(bundle: Path) -> set[str]:
    files: set[str] = set()
    for path in bundle.rglob("*"):
        if path.is_symlink():
            raise ModelBundleError(f"bundle symlinks are not allowed: {path.relative_to(bundle)}")
        if path.is_file() and path.name != METADATA_NAME:
            files.add(path.relative_to(bundle).as_posix())
    return files


def _load_metadata(bundle: Path) -> dict[str, Any]:
    path = bundle / METADATA_NAME
    if not path.is_file():
        raise ModelBundleError(f"missing {METADATA_NAME}: {path}")
    try:
        metadata = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ModelBundleError(f"invalid {METADATA_NAME}: {exc}") from exc
    if not isinstance(metadata, dict):
        raise ModelBundleError(f"{METADATA_NAME} must contain an object")
    return metadata


def _validate_header(metadata: dict[str, Any]) -> Mapping[str, object]:
    if metadata.get("schema_version") != SCHEMA_VERSION:
        raise ModelBundleError(
            f"unsupported bundle schema_version: {metadata.get('schema_version')!r}"
        )
    if metadata.get("bundle_type") != BUNDLE_TYPE:
        raise ModelBundleError(f"unexpected bundle_type: {metadata.get('bundle_type')!r}")
    if metadata.get("entrypoints") != ENTRYPOINTS:
        raise ModelBundleError("bundle entrypoints do not match the sim contract")
    records = metadata.get("files")
    if not isinstance(records, dict) or not records:
        raise ModelBundleError("bundle files manifest is empty or invalid")
    identity = dict(metadata)
    bundle_id = identity.pop("bundle_id", None)
    if not isinstance(bundle_id, str) or bundle_id != _canonical_sha256(identity):
        raise ModelBundleError("bundle_id does not match metadata")
    return records


def _validate_payload(bundle: Path, records: Mapping[str, object]) -> None:
    expected: set[str] = set()
    for raw_name, raw_record in records.items():
        relative = _relative_path(raw_name, context="manifest file")
        name = relative.as_posix()
        expected.add(name)
        if not isinstance(raw_record, dict):
            raise ModelBundleError(f"invalid file record: {name}")
        path = bundle / relative
        if path.is_symlink() or not path.is_file():
            raise ModelBundleError(f"missing payload file: {name}")
        expected_hash = raw_record.get("sha256")
        if not isinstance(expected_hash, str) or _sha256(path) != expected_hash:
            raise ModelBundleError(f"hash mismatch: {name}")
        expected_size = raw_record.get("size")
        if not isinstance(expected_size, int) or path.stat().st_size != expected_size:
            raise ModelBundleError(f"size mismatch: {name}")

    actual = _payload_files(bundle)
    untracked = sorted(actual - expected)
    absent = sorted(expected - actual)
    if untracked:
        raise ModelBundleError("untracked files: " + ", ".join(untracked))
    if absent:
        raise ModelBundleError("missing files: " + ", ".join(absent))


def _require_reference(bundle: Path, raw: object, *, context: str) -> None:
    relative = _relative_path(raw, context=context)
    path = bundle / relative
    if path.is_symlink() or not path.is_file():
        raise ModelBundleError(f"missing {context}: {relative.as_posix()}")


def _validate_blueprint(bundle: Path) -> None:
    path = bundle / ENTRYPOINTS["blueprint"]
    try:
        blueprint = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ModelBundleError(f"invalid blueprint: {exc}") from exc
    parts = blueprint.get("parts") if isinstance(blueprint, dict) else None
    if not isinstance(parts, list) or not parts:
        raise ModelBundleError("blueprint parts are empty or invalid")
    for index, part in enumerate(parts):
        assets = part.get("assets") if isinstance(part, dict) else None
        if not isinstance(assets, dict):
            raise ModelBundleError(f"blueprint part {index} has no assets object")
        for asset_type in ("mesh", "frame", "physics"):
            _require_reference(
                bundle,
                assets.get(asset_type),
                context=f"blueprint part {index} {asset_type}",
            )


def _validate_urdf(bundle: Path, name: str) -> None:
    path = bundle / name
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ModelBundleError(f"invalid URDF {name}: {exc}") from exc
    if root.tag != "robot":
        raise ModelBundleError(f"URDF root is not robot: {name}")
    for index, mesh in enumerate(root.iter("mesh")):
        _require_reference(
            bundle,
            mesh.attrib.get("filename"),
            context=f"{name} mesh {index}",
        )


def validate_model_bundle(bundle_dir: str | os.PathLike[str]) -> dict[str, Any]:
    """Validate a model bundle without importing Genesis or the model builder."""

    bundle = Path(bundle_dir).resolve()
    if not bundle.is_dir():
        raise ModelBundleError(f"bundle directory not found: {bundle}")
    metadata = _load_metadata(bundle)
    records = _validate_header(metadata)
    _validate_payload(bundle, records)
    _validate_blueprint(bundle)
    _validate_urdf(bundle, ENTRYPOINTS["arm_urdf"])
    _validate_urdf(bundle, ENTRYPOINTS["robot_urdf"])
    return metadata


def _checked_bundle_path(raw_path: str | os.PathLike[str]) -> Path:
    bundle = Path(raw_path).expanduser().resolve()
    if not bundle.is_dir():
        raise ModelBundleError(f"bundle directory not found: {bundle}")
    validate_model_bundle(bundle)
    return bundle


def _discovery_roots() -> tuple[Path, ...]:
    roots: list[Path] = []
    for start in (Path.cwd().resolve(), Path(__file__).resolve().parent):
        for root in (start, *start.parents):
            if root not in roots:
                roots.append(root)
    return tuple(roots)


def resolve_model_bundle(
    bundle_dir: str | os.PathLike[str] | None = None,
) -> Path:
    """Resolve and validate the immutable bundle used by this process.

    Explicit CLI/config input wins, followed by ``ELESIM_MODEL_BUNDLE``.  The
    final search supports both a source checkout and an unpacked release whose
    working directory is at or below the deployment root.
    """

    if bundle_dir is not None and str(bundle_dir).strip():
        return _checked_bundle_path(bundle_dir)

    configured = os.environ.get(MODEL_BUNDLE_ENV, "").strip()
    if configured:
        return _checked_bundle_path(configured)

    for root in _discovery_roots():
        for relative in (
            "data/models/assemblies/zed-mini",
            "payload/data/models/assemblies/zed-mini",
        ):
            candidate = root / relative
            if (candidate / METADATA_NAME).is_file():
                return _checked_bundle_path(candidate)

    raise ModelBundleError(
        "ZED Mini model bundle not found; pass --model-bundle or set "
        f"{MODEL_BUNDLE_ENV}"
    )


def resolve_camera_profile_bundle(
    profile: str,
    bundle_dir: str | os.PathLike[str] | None = None,
) -> Path:
    """Resolve the immutable bundle required by a selected camera profile."""
    selected = camera_profile(profile)
    expected_name = selected.model_bundle
    if bundle_dir is not None and str(bundle_dir).strip():
        candidate = resolve_model_bundle(bundle_dir)
        if candidate.name != expected_name:
            raise ModelBundleError(
                f"camera profile {selected.name!r} requires model bundle "
                f"{expected_name!r}, got {candidate.name!r}"
            )
        return candidate

    configured = os.environ.get(MODEL_BUNDLE_ENV, "").strip()
    if configured:
        base = resolve_model_bundle(configured)
        if (
            base.name in {item.model_bundle for item in CAMERA_PROFILES.values()}
            and base.name != expected_name
        ):
            raise ModelBundleError(
                f"camera profile {selected.name!r} requires model bundle "
                f"{expected_name!r}, got {base.name!r}"
            )
        candidate = base if base.name == expected_name else base.parent / expected_name
        return _checked_bundle_path(candidate)

    for root in _discovery_roots():
        for relative in ("data/models/assemblies", "payload/data/models/assemblies"):
            candidate = root / relative / expected_name
            if (candidate / METADATA_NAME).is_file():
                return _checked_bundle_path(candidate)
    raise ModelBundleError(
        f"model bundle {expected_name!r} for camera profile {selected.name!r} not found"
    )


__all__ = [
    "MODEL_BUNDLE_ENV",
    "ModelBundleError",
    "resolve_camera_profile_bundle",
    "resolve_model_bundle",
    "validate_model_bundle",
]
